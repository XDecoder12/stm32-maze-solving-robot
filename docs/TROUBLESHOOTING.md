# Line-tracking troubleshooting

## Reported behavior

The robot works on white vinyl tape over black chart paper but fails even on straight sections of a fully printed vinyl maze with a 30 mm white line. Calibration is performed, but the cause is not yet confirmed.

## Diagnostic sequence

1. Preserve the working firmware before any experiments.
2. Disable motor output while collecting sensor measurements.
3. At the normal sensor mounting height, record at least ten raw ADC samples from each of the eight channels on: original black paper, original white tape, printed black vinyl and printed white vinyl.
4. Compare white and black distributions **per channel** and verify which ADC polarity corresponds to the intended line.
5. Check calibration extrema and resulting thresholds. The current main sketch appears to initialize minima to 0 and maxima to 4095, preventing updates for ordinary 12-bit ADC values.
6. Once line classification is reliable, test low-speed straight-line following.
7. Only then investigate junction classification, encoder offsets, wheel traction and PID gains.

## Suggested log format

```csv
surface,region,sample,s1,s2,s3,s4,s5,s6,s7,s8
chart-paper,black,1,,,,,,,,
white-tape,white,1,,,,,,,,
printed-vinyl,black,1,,,,,,,,
printed-vinyl,white,1,,,,,,,,
```

Record firmware variant, ADC resolution, supply voltage, sensor height and line width alongside the measurements.

**Note:** This is a diagnostic plan, not a hardware-tested solution.
