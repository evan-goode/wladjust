use std::{
    env, fmt,
    fs::OpenOptions,
    io::{Read, Seek, SeekFrom, Write},
    num::NonZeroU32,
    path::PathBuf,
    process::Command,
    str::FromStr,
    sync::mpsc,
    thread,
    time::{Duration, Instant},
};

use clap::Parser;
use libnotify;
use libnotify_sys::NOTIFY_EXPIRES_NEVER;
use log::{LevelFilter, debug};
use nix::{
    fcntl::{Flock, FlockArg},
    sys::signal::{Signal, kill},
    unistd::{Pid, getpid},
};
use signal_hook::{consts::signal::*, iterator::Signals};
use smithay_client_toolkit::{
    compositor::{CompositorHandler, CompositorState},
    delegate_compositor, delegate_keyboard, delegate_layer, delegate_output, delegate_pointer,
    delegate_pointer_constraints, delegate_registry, delegate_relative_pointer, delegate_seat,
    delegate_shm,
    output::{OutputHandler, OutputState},
    registry::{ProvidesRegistryState, RegistryState},
    registry_handlers,
    seat::{
        Capability, SeatHandler, SeatState,
        keyboard::{KeyEvent, KeyboardHandler, Keysym, Modifiers, RawModifiers},
        pointer::{PointerEvent, PointerEventKind, PointerHandler},
        pointer_constraints::{PointerConstraintsHandler, PointerConstraintsState},
        relative_pointer::{RelativeMotionEvent, RelativePointerHandler, RelativePointerState},
    },
    shell::{
        WaylandSurface,
        wlr_layer::{
            Anchor, KeyboardInteractivity, Layer, LayerShell, LayerShellHandler, LayerSurface,
            LayerSurfaceConfigure,
        },
    },
    shm::{Shm, ShmHandler, slot::SlotPool},
};
use wayland_client::{
    Connection, QueueHandle,
    globals::registry_queue_init,
    protocol::{wl_keyboard, wl_output, wl_pointer, wl_seat, wl_shm, wl_surface},
};
use wayland_protocols::wp::{
    pointer_constraints::zv1::client::{
        zwp_confined_pointer_v1, zwp_locked_pointer_v1, zwp_pointer_constraints_v1,
    },
    relative_pointer::zv1::client::zwp_relative_pointer_v1,
};

const APPLICATION_NAME: &str = "wladjust";

type DynError = Box<dyn std::error::Error + Send + Sync + 'static>;

#[derive(Copy, Clone, Debug, PartialEq)]
enum Value {
    Int(i64),
    Float(f64),
}

impl FromStr for Value {
    type Err = DynError;
    fn from_str(s: &str) -> Result<Self, DynError> {
        if s.contains('.') {
            Ok(Value::Float(s.parse::<f64>()?))
        } else {
            Ok(Value::Int(s.parse::<i64>()?))
        }
    }
}

impl fmt::Display for Value {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Value::Int(i) => write!(f, "{}", i),
            Value::Float(fl) => write!(f, "{}", fl),
        }
    }
}

impl From<Value> for f64 {
    fn from(v: Value) -> Self {
        match v {
            Value::Int(i) => i as f64,
            Value::Float(f) => f,
        }
    }
}

impl From<Value> for i64 {
    fn from(v: Value) -> Self {
        match v {
            Value::Int(i) => i,
            Value::Float(f) => f.round() as i64,
        }
    }
}

impl From<f64> for Value {
    fn from(f: f64) -> Value {
        Value::Float(f)
    }
}

impl From<i64> for Value {
    fn from(i: i64) -> Value {
        Value::Int(i)
    }
}

fn replace_placeholders(template: &str, value: &str) -> Result<(String, u64), DynError> {
    let mut result = String::new();
    let mut count = 0;
    let mut chars = template.chars().peekable();
    while let Some(c) = chars.next() {
        match c {
            '{' => match chars.peek() {
                Some('{') => {
                    result.push('{');
                    chars.next();
                }
                Some('}') => {
                    result.push_str(value);
                    count += 1;
                    chars.next();
                }
                _ => {
                    return Err("unmatched '{'".into());
                }
            },
            '}' => match chars.peek() {
                Some('}') => {
                    result.push('}');
                    chars.next();
                }
                _ => {
                    return Err("unmatched '}'".into());
                }
            },
            c => {
                result.push(c);
            }
        }
    }
    Ok((result, count))
}

fn write_value(args: &Args, value: Value) -> Result<(), DynError> {
    let start = Instant::now();

    let (arg0, _) = replace_placeholders(&args.command[0], &value.to_string())?;
    let mut command = Command::new(&arg0);
    let mut cmdline = vec![arg0.clone()];
    for arg in &args.command[1..] {
        let (replaced, _) = replace_placeholders(arg, &value.to_string())?;
        command.arg(&replaced);
        cmdline.push(replaced);
    }

    let output = command.output()?;

    let elapsed = start.elapsed();

    if !output.status.success() {
        if let Some(code) = output.status.code() {
            return Err(format!(
                "command {:?} exited with code {:?}",
                cmdline.join(" "),
                code
            )
            .into());
        } else {
            return Err(format!("process \"{}\" terminated by signal", &arg0).into());
        }
    }

    thread::sleep(
        args.debounce
            .checked_sub(elapsed)
            .unwrap_or(Duration::from_secs(0)),
    );

    Ok(())
}

fn pointer_to_eased(pointer: f64, pointer_range: f64) -> f64 {
    pointer / pointer_range
}

fn value_to_eased(value: Value, min: Value, max: Value, exponent: f64) -> f64 {
    let minf: f64 = min.into();
    let maxf: f64 = max.into();
    ((f64::from(value) - minf) / (maxf - minf)).powf(1.0 / exponent)
}

fn eased_to_value(eased: f64, min: Value, max: Value, exponent: f64) -> Value {
    let minf: f64 = min.into();
    let maxf: f64 = max.into();
    let valuef = minf + eased.powf(exponent) * (maxf - minf);
    match (&min, &max) {
        (Value::Int(_), Value::Int(_)) => Value::Int(valuef.round() as i64),
        _ => Value::Float(valuef),
    }
}

fn eased_to_pointer(eased: f64, pointer_range: f64) -> f64 {
    eased * pointer_range
}

fn notification_body(eased: f64) -> String {
    format!("{}%", (100.0 * eased).ceil() as u64)
}

fn parse_millis(s: &str) -> Result<Duration, DynError> {
    let ms: u64 = s.parse()?;
    Ok(Duration::from_millis(ms))
}

enum Message {
    Exit,
    Notify(f64),
    Err(DynError),
    Signal(i32),
}

fn output(args: Args, rx_value: mpsc::Receiver<Value>, tx: mpsc::Sender<Message>) {
    let mut last_value: Option<Value> = None;
    loop {
        let mut value = match rx_value.recv() {
            Ok(v) => v,
            Err(_) => return,
        };

        // Drop stale messages
        while let Ok(v) = rx_value.try_recv() {
            value = v;
        }

        if Some(value) != last_value {
            last_value = Some(value);

            if let Err(err) = write_value(&args, value) {
                tx.send(Message::Err(err)).unwrap();
                return;
            }
        }
    }
}

#[derive(Clone, Parser, Debug)]
#[command(version, about, long_about = "Wayland program to smoothly adjust anything using the pointer.")]
struct Args {
    /// Enable debug logging
    #[arg(long, default_value_t = false)]
    debug: bool,

    /// The tag is shown as the notification title. If another process has the same tag, both the
    /// old and the new processes will be killed.
    #[arg(long)]
    tag: Option<String>,

    /// Minimum value to pass to the command. If both --min-value and --max-value are integers, the
    /// value will be passed as an integer. Otherwise, the value will be passed as a float.
    #[arg(long, default_value = "0.0")]
    min_value: Value,

    /// Maximum value to pass to the command. If both --min-value and --max-value are integers, the
    /// value will be passed as an integer. Otherwise, the value will be passed as a float.
    #[arg(long, default_value = "1.0")]
    max_value: Value,

    /// The degree of the polynomial along which to adjust. Default is linear. Use a higher value
    /// to get more resolution towards the lower end of the range.
    #[arg(long, default_value = "1.0")]
    exponent: f64,

    /// Number of pixels the pointer needs to move to cover the entire range
    #[arg(long, default_value = "1000.0")]
    pointer_range: f64,

    /// Minimum number of milliseconds between executions of the command
    #[arg(long, value_parser = parse_millis, default_value = "0")]
    debounce: Duration,

    /// Value from which to start adjusting
    #[arg(long, required = true)]
    current_value: Value,

    /// Command to call. {} will be replaced with the adjusted value. Use {{ or }} to escape
    /// literal braces.
    #[arg(last = true, num_args = 1.., required = true)]
    command: Vec<String>,
}

struct AutoClosedNotification {
    inner: libnotify::Notification,
}

impl Drop for AutoClosedNotification {
    fn drop(&mut self) {
        let _ = self.inner.close();
    }
}

struct SimpleLayer {
    registry_state: RegistryState,
    seat_state: SeatState,
    output_state: OutputState,
    relative_pointer_state: RelativePointerState,
    pointer_constraint_state: PointerConstraintsState,
    shm: Shm,

    first_configure: bool,
    pool: SlotPool,
    width: u32,
    height: u32,
    layer: LayerSurface,
    keyboard: Option<wl_keyboard::WlKeyboard>,
    pointer: Option<wl_pointer::WlPointer>,
    relative_pointer: Option<zwp_relative_pointer_v1::ZwpRelativePointerV1>,

    args: Args,
    tx: mpsc::Sender<Message>,
    tx_value: mpsc::Sender<Value>,
    pointer_pos: (f64, f64),
    min_pointer: f64,
    max_pointer: f64,
}

fn main() -> Result<(), DynError> {
    let args = Args::parse();

    let level = if args.debug {
        LevelFilter::Debug
    } else {
        LevelFilter::Info
    };
    env_logger::Builder::new().filter_level(level).init();

    // Validate arguments
    let mut any_placeholder = false;
    for arg in args.command.iter() {
        let (_, count) = replace_placeholders(arg, "")?;
        any_placeholder |= count > 00;
    }
    if !any_placeholder {
        return Err("command must contain at least one occurrence of '{}'".into());
    }

    // Locking system to make sure only one instance is running at a time
    let runtime_dir = env::var("XDG_RUNTIME_DIR").unwrap_or_else(|_| "/tmp".into());
    let path = PathBuf::from(runtime_dir).join(APPLICATION_NAME);
    std::fs::create_dir_all(&path)?;
    let owner_path = path.join("owner.lock");
    let startup_path = path.join("startup.lock");

    let mut owner_fd = OpenOptions::new()
        .read(true)
        .write(true)
        .create(true)
        .open(&owner_path)
        .map_err(|e| format!("failed to open {}: {}", owner_path.display(), e))?;
    let startup_fd = OpenOptions::new()
        .write(true)
        .create(true)
        .open(&startup_path)
        .map_err(|e| format!("failed to open {}: {}", startup_path.display(), e))?;

    // Secondary "startup" lock while ownership is being transferring from one process to another
    let startup_lock = Flock::lock(startup_fd, FlockArg::LockExclusive).map_err(|e| {
        format!(
            "failed to acquire lock on {}: {}",
            startup_path.display(),
            e.1
        )
    })?;

    let _owner_lock = match Flock::lock(
        owner_fd.try_clone().unwrap(),
        FlockArg::LockExclusiveNonblock,
    ) {
        Ok(l) => l,
        _ => {
            // Another process has the owner lock
            let mut owner_data = String::new();
            owner_fd.read_to_string(&mut owner_data)?;
            let (old_pid, old_tag) = owner_data.split_once('\n').unwrap();
            // Kill the old owner
            let _ = kill(Pid::from_raw(old_pid.parse().unwrap()), Signal::SIGTERM);
            // And seize the lock
            let owner_lock = Flock::lock(owner_fd.try_clone()?, FlockArg::LockExclusive).unwrap();

            // If the old owner had the same tag, exit. Makes it easy to "toggle" wladjust on and
            // off.
            if old_tag == args.tag.as_deref().unwrap_or("") {
                return Ok(());
            }
            owner_lock
        }
    };

    // Write our pid and tag to the owner lockfile
    owner_fd.set_len(0)?;
    owner_fd.seek(SeekFrom::Start(0))?;
    owner_fd
        .write_all(format!("{}\n{}", getpid(), args.tag.as_deref().unwrap_or("")).as_bytes())?;
    owner_fd.flush()?;
    owner_fd.sync_all()?;

    startup_lock.unlock().unwrap();

    // Main channel for inter-thread communication
    let (tx, rx) = mpsc::channel();

    // Wayland
    let conn = Connection::connect_to_env()?;

    let (globals, mut event_queue) = registry_queue_init(&conn)?;
    let qh = event_queue.handle();

    let compositor = CompositorState::bind(&globals, &qh)
        .map_err(|e| format!("wl_compositor is not available: {}", e))?;
    let layer_shell = LayerShell::bind(&globals, &qh)
        .map_err(|e| format!("layer shell is not available: {}", e))?;
    let shm = Shm::bind(&globals, &qh).map_err(|e| format!("wl_shm is not available: {}", e))?;

    let surface = compositor.create_surface(&qh);

    let layer =
        layer_shell.create_layer_surface(&qh, surface, Layer::Overlay, Some("simple_layer"), None);
    layer.set_anchor(Anchor::TOP | Anchor::RIGHT | Anchor::BOTTOM | Anchor::LEFT);
    layer.set_keyboard_interactivity(KeyboardInteractivity::OnDemand);

    layer.commit();

    let pool = SlotPool::new(1, &shm).map_err(|e| format!("failed to create pool: {}", e))?;

    // Set up pointer space
    let min_pointer = eased_to_pointer(0.0, args.pointer_range);
    let max_pointer = eased_to_pointer(1.0, args.pointer_range);
    let initial_eased = value_to_eased(
        args.current_value,
        args.min_value,
        args.max_value,
        args.exponent,
    );
    let current_pointer = eased_to_pointer(initial_eased, args.pointer_range);

    // libnotify
    libnotify::init(APPLICATION_NAME).map_err(|e| format!("couldn't init libnotify: {}", e))?;
    let notification_title = args.tag.as_deref().unwrap_or(APPLICATION_NAME);
    let notification = AutoClosedNotification {
        inner: libnotify::Notification::new(
            notification_title,
            Some(notification_body(initial_eased)).as_deref(),
            None,
        ),
    };
    notification.inner.set_timeout(NOTIFY_EXPIRES_NEVER);
    notification.inner.show()?;

    // Run output in another thread so it doesn't block everything else
    let (tx_value, rx_value) = mpsc::channel();
    let tx_clone = tx.clone();
    let args_clone = args.clone();
    thread::spawn(move || {
        output(args_clone, rx_value, tx_clone);
    });

    let mut simple_layer = SimpleLayer {
        registry_state: RegistryState::new(&globals),
        seat_state: SeatState::new(&globals, &qh),
        output_state: OutputState::new(&globals, &qh),
        relative_pointer_state: RelativePointerState::bind(&globals, &qh),
        pointer_constraint_state: PointerConstraintsState::bind(&globals, &qh),
        shm,

        first_configure: true,
        pool,
        width: 1,
        height: 1,
        layer,
        keyboard: None,
        pointer: None,
        relative_pointer: None,

        tx: tx.clone(),
        tx_value,
        pointer_pos: (current_pointer, 0.0),
        min_pointer: min_pointer,
        max_pointer: max_pointer,
        args: args.clone(),
    };

    // Run wayland event queue in another thread
    thread::spawn(move || {
        loop {
            event_queue.blocking_dispatch(&mut simple_layer).unwrap();
        }
    });

    // Set up signal handlers
    let mut signals = Signals::new([SIGINT, SIGTERM])?;
    let tx_clone = tx.clone();
    thread::spawn(move || {
        for sig in signals.forever() {
            tx_clone.send(Message::Signal(sig)).ok();
        }
    });

    loop {
        match rx.recv() {
            Ok(message) => match message {
                Message::Exit => {
                    return Ok(());
                }
                Message::Signal(signal) => {
                    return Err(format!("caught signal {}", signal).into());
                }
                Message::Err(err) => {
                    return Err(err);
                }
                Message::Notify(eased) => {
                    notification.inner.update(
                        notification_title,
                        Some(notification_body(eased)).as_deref(),
                        None,
                    )?;
                    notification.inner.show()?;
                }
            },
            Err(err) => {
                return Err(err.into());
            }
        };
    }
}

impl CompositorHandler for SimpleLayer {
    fn scale_factor_changed(
        &mut self,
        _conn: &Connection,
        _qh: &QueueHandle<Self>,
        _surface: &wl_surface::WlSurface,
        _new_factor: i32,
    ) {
    }

    fn transform_changed(
        &mut self,
        _conn: &Connection,
        _qh: &QueueHandle<Self>,
        _surface: &wl_surface::WlSurface,
        _new_transform: wl_output::Transform,
    ) {
    }

    fn frame(
        &mut self,
        _conn: &Connection,
        _qh: &QueueHandle<Self>,
        _surface: &wl_surface::WlSurface,
        _time: u32,
    ) {
    }

    fn surface_enter(
        &mut self,
        _conn: &Connection,
        _qh: &QueueHandle<Self>,
        _surface: &wl_surface::WlSurface,
        _output: &wl_output::WlOutput,
    ) {
    }

    fn surface_leave(
        &mut self,
        _conn: &Connection,
        _qh: &QueueHandle<Self>,
        _surface: &wl_surface::WlSurface,
        _output: &wl_output::WlOutput,
    ) {
    }
}

impl OutputHandler for SimpleLayer {
    fn output_state(&mut self) -> &mut OutputState {
        &mut self.output_state
    }

    fn new_output(
        &mut self,
        _conn: &Connection,
        _qh: &QueueHandle<Self>,
        _output: wl_output::WlOutput,
    ) {
    }

    fn update_output(
        &mut self,
        _conn: &Connection,
        _qh: &QueueHandle<Self>,
        _output: wl_output::WlOutput,
    ) {
    }

    fn output_destroyed(
        &mut self,
        _conn: &Connection,
        _qh: &QueueHandle<Self>,
        _output: wl_output::WlOutput,
    ) {
    }
}

impl LayerShellHandler for SimpleLayer {
    fn closed(&mut self, _conn: &Connection, _qh: &QueueHandle<Self>, _layer: &LayerSurface) {
        self.tx.send(Message::Exit).unwrap();
    }

    fn configure(
        &mut self,
        _conn: &Connection,
        qh: &QueueHandle<Self>,
        _layer: &LayerSurface,
        configure: LayerSurfaceConfigure,
        _serial: u32,
    ) {
        self.width = NonZeroU32::new(configure.new_size.0).map_or(1, NonZeroU32::get);
        self.height = NonZeroU32::new(configure.new_size.1).map_or(1, NonZeroU32::get);

        // Initiate the first draw.
        if self.first_configure {
            self.first_configure = false;
            self.draw(qh);
        }
    }
}

impl SeatHandler for SimpleLayer {
    fn seat_state(&mut self) -> &mut SeatState {
        &mut self.seat_state
    }

    fn new_seat(&mut self, _: &Connection, _: &QueueHandle<Self>, _: wl_seat::WlSeat) {}

    fn new_capability(
        &mut self,
        _conn: &Connection,
        qh: &QueueHandle<Self>,
        seat: wl_seat::WlSeat,
        capability: Capability,
    ) {
        if capability == Capability::Keyboard && self.keyboard.is_none() {
            debug!("Set keyboard capability");
            let Ok(keyboard) = self.seat_state.get_keyboard(qh, &seat, None) else {
                self.tx
                    .send(Message::Err("failed to create keyboard".into()))
                    .unwrap();
                return;
            };
            self.keyboard = Some(keyboard);
        }

        if capability == Capability::Pointer && self.pointer.is_none() {
            debug!("Set pointer capability");
            let Ok(pointer) = self.seat_state.get_pointer(qh, &seat) else {
                self.tx
                    .send(Message::Err("failed to create pointer".into()))
                    .unwrap();
                return;
            };

            let Ok(relative_pointer) = self
                .relative_pointer_state
                .get_relative_pointer(&pointer, qh)
            else {
                self.tx
                    .send(Message::Err(
                        "compositor does not support pointer events".into(),
                    ))
                    .unwrap();
                return;
            };

            let surface = self.layer.wl_surface();

            if let Err(_) = self.pointer_constraint_state.lock_pointer(
                surface,
                &pointer,
                None,
                zwp_pointer_constraints_v1::Lifetime::Persistent,
                qh,
            ) {
                self.tx
                    .send(Message::Err(
                        "compositor does not support pointer events".into(),
                    ))
                    .unwrap();
                return;
            }

            self.pointer = Some(pointer);
            self.relative_pointer = Some(relative_pointer);
        }
    }

    fn remove_capability(
        &mut self,
        _conn: &Connection,
        _: &QueueHandle<Self>,
        _: wl_seat::WlSeat,
        capability: Capability,
    ) {
        if capability == Capability::Keyboard && self.keyboard.is_some() {
            debug!("Unset keyboard capability");
            self.keyboard.take().unwrap().release();
        }

        if capability == Capability::Pointer && self.pointer.is_some() {
            debug!("Unset pointer capability");
            self.pointer.take().unwrap().release();
        }
    }

    fn remove_seat(&mut self, _: &Connection, _: &QueueHandle<Self>, _: wl_seat::WlSeat) {}
}

impl KeyboardHandler for SimpleLayer {
    fn enter(
        &mut self,
        _: &Connection,
        _: &QueueHandle<Self>,
        _: &wl_keyboard::WlKeyboard,
        surface: &wl_surface::WlSurface,
        _: u32,
        _: &[u32],
        keysyms: &[Keysym],
    ) {
        if self.layer.wl_surface() == surface {
            debug!("Keyboard focus on window with pressed syms: {keysyms:?}");
        }
    }

    fn leave(
        &mut self,
        _: &Connection,
        _: &QueueHandle<Self>,
        _: &wl_keyboard::WlKeyboard,
        surface: &wl_surface::WlSurface,
        _: u32,
    ) {
        if self.layer.wl_surface() == surface {
            debug!("Release keyboard focus on window");
        }
    }

    fn press_key(
        &mut self,
        _conn: &Connection,
        _qh: &QueueHandle<Self>,
        _: &wl_keyboard::WlKeyboard,
        _: u32,
        event: KeyEvent,
    ) {
        debug!("Key press: {event:?}");
        self.tx.send(Message::Exit).unwrap();
    }

    fn repeat_key(
        &mut self,
        _conn: &Connection,
        _qh: &QueueHandle<Self>,
        _keyboard: &wl_keyboard::WlKeyboard,
        _serial: u32,
        _event: KeyEvent,
    ) {
    }

    fn release_key(
        &mut self,
        _: &Connection,
        _: &QueueHandle<Self>,
        _: &wl_keyboard::WlKeyboard,
        _: u32,
        _event: KeyEvent,
    ) {
    }

    fn update_modifiers(
        &mut self,
        _: &Connection,
        _: &QueueHandle<Self>,
        _: &wl_keyboard::WlKeyboard,
        _serial: u32,
        _modifiers: Modifiers,
        _raw_modifiers: RawModifiers,
        _layout: u32,
    ) {
    }
}

impl PointerHandler for SimpleLayer {
    fn pointer_frame(
        &mut self,
        _conn: &Connection,
        _qh: &QueueHandle<Self>,
        _pointer: &wl_pointer::WlPointer,
        events: &[PointerEvent],
    ) {
        use PointerEventKind::*;
        for event in events {
            if &event.surface != self.layer.wl_surface() {
                continue;
            }
            match event.kind {
                Enter { .. } => {
                    debug!("Pointer entered @{:?}", event.position);
                }
                Leave { .. } => {
                    debug!("Pointer left");
                }
                Press { button, .. } => {
                    debug!("Press {:x} @ {:?}", button, event.position);
                    self.tx.send(Message::Exit).unwrap();
                }
                _ => {}
            }
        }
    }
}

impl RelativePointerHandler for SimpleLayer {
    fn relative_pointer_motion(
        &mut self,
        _conn: &Connection,
        _qh: &QueueHandle<Self>,
        _relative_pointer: &zwp_relative_pointer_v1::ZwpRelativePointerV1,
        _pointer: &wl_pointer::WlPointer,
        event: RelativeMotionEvent,
    ) {
        self.pointer_pos.0 =
            (self.pointer_pos.0 + event.delta.0).clamp(self.min_pointer, self.max_pointer);

        let eased = pointer_to_eased(self.pointer_pos.0, self.args.pointer_range);
        let value = eased_to_value(
            eased,
            self.args.min_value,
            self.args.max_value,
            self.args.exponent,
        );

        let _ = self.tx_value.send(value);
        self.tx.send(Message::Notify(eased)).unwrap();
    }
}

impl PointerConstraintsHandler for SimpleLayer {
    fn confined(
        &mut self,
        _conn: &Connection,
        _qh: &QueueHandle<Self>,
        _confined_pointer: &zwp_confined_pointer_v1::ZwpConfinedPointerV1,
        _surface: &wl_surface::WlSurface,
        _pointer: &wl_pointer::WlPointer,
    ) {
    }

    fn unconfined(
        &mut self,
        _conn: &Connection,
        _qh: &QueueHandle<Self>,
        _confined_pointer: &zwp_confined_pointer_v1::ZwpConfinedPointerV1,
        _surface: &wl_surface::WlSurface,
        _pointer: &wl_pointer::WlPointer,
    ) {
    }

    fn locked(
        &mut self,
        _conn: &Connection,
        _qh: &QueueHandle<Self>,
        _locked_pointer: &zwp_locked_pointer_v1::ZwpLockedPointerV1,
        _surface: &wl_surface::WlSurface,
        _pointer: &wl_pointer::WlPointer,
    ) {
        debug!("Pointer locked");
    }

    fn unlocked(
        &mut self,
        _conn: &Connection,
        _qh: &QueueHandle<Self>,
        _locked_pointer: &zwp_locked_pointer_v1::ZwpLockedPointerV1,
        _surface: &wl_surface::WlSurface,
        _pointer: &wl_pointer::WlPointer,
    ) {
        debug!("Pointer unlocked");
    }
}

impl ShmHandler for SimpleLayer {
    fn shm_state(&mut self) -> &mut Shm {
        &mut self.shm
    }
}

impl SimpleLayer {
    pub fn draw(&mut self, _qh: &QueueHandle<Self>) {
        let width = self.width;
        let height = self.height;
        let stride = self.width as i32 * 4;

        let (buffer, _) = self
            .pool
            .create_buffer(
                width as i32,
                height as i32,
                stride,
                wl_shm::Format::Argb8888,
            )
            .expect("create buffer");

        buffer
            .attach_to(self.layer.wl_surface())
            .expect("buffer attach");
        self.layer.commit();
    }
}

delegate_compositor!(SimpleLayer);
delegate_output!(SimpleLayer);
delegate_shm!(SimpleLayer);

delegate_seat!(SimpleLayer);
delegate_keyboard!(SimpleLayer);
delegate_pointer!(SimpleLayer);
delegate_relative_pointer!(SimpleLayer);
delegate_pointer_constraints!(SimpleLayer);

delegate_layer!(SimpleLayer);

delegate_registry!(SimpleLayer);

impl ProvidesRegistryState for SimpleLayer {
    fn registry(&mut self) -> &mut RegistryState {
        &mut self.registry_state
    }
    registry_handlers![OutputState, SeatState];
}
