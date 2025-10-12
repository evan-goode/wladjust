# wladjust

Wayland program to smoothly adjust anything using the pointer.

1. Bind a key to `wladjust` (see examples below). Press the key.
2. Move the cursor left or right to smoothly adjust the value lower or higher.
3. Click or press any key to exit

## Examples

### Brightness with `brightnessctl`

```
wladjust \
    --tag Brightness \
    --min-value 1 \
    --max-value "$(brightnessctl --class=backlight max)" \
    --exponent 3 \
    --current-value "$(brightnessctl --class=backlight get)" \
    --debounce 100 \
    -- brightnessctl --class=backlight set {}
```

### Volume with `wpctl`

```
wladjust \
    --tag Volume \
    --current-value "$(wpctl get-volume @DEFAULT_AUDIO_SINK@ | cut -d ' ' -f 2)" \
    -- wpctl set-volume --limit 1.0 @DEFAULT_AUDIO_SINK@ {}
```

## "Toggling" behavior

If you start `wladjust` and another `wladjust` process is already running, the old process will be killed. If both processes have the same `--tag`, both will be killed. So you can bind a key to `wladjust --tag foo` and pressing it multiple times will toggle `wladjust`.

## Usage

```
Usage: wladjust [OPTIONS] --current-value <CURRENT_VALUE> -- <COMMAND>...

Arguments:
  <COMMAND>...
          Command to call. {} will be replaced with the adjusted value. Use {{ or }} to escape literal braces

Options:
      --debug
          Enable debug logging

      --tag <TAG>
          The tag is shown as the notification title. If another process has the same tag, both the old and the new processes will be killed

      --min-value <MIN_VALUE>
          Minimum value to pass to the command. If both --min-value and --max-value are integers, the value will be passed as an integer. Otherwise, the value will be passed as a float
          
          [default: 0.0]

      --max-value <MAX_VALUE>
          Maximum value to pass to the command. If both --min-value and --max-value are integers, the value will be passed as an integer. Otherwise, the value will be passed as a float
          
          [default: 1.0]

      --exponent <EXPONENT>
          The degree of the polynomial along which to adjust. Default is linear. Use a higher value to get more resolution towards the lower end of the range
          
          [default: 1.0]

      --pointer-range <POINTER_RANGE>
          Number of pixels the pointer needs to move to cover the entire range
          
          [default: 1000.0]

      --debounce <DEBOUNCE>
          Minimum number of milliseconds between executions of the command
          
          [default: 0]

      --current-value <CURRENT_VALUE>
          Value from which to start adjusting

  -h, --help
          Print help (see a summary with '-h')

  -V, --version
          Print version
```

## Building

```
nix build
```

## License

GPLv3
