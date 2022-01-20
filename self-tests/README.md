# lpc546 HAL tests

The crates assumes that you'll test the hal on a LPCXpresso54608 Dev board.

You will need to connect 
* D7 (J13:1 / P3.01) <-> 3V3
* D6 (J13:3 / P1.22) <-> GND
* J9:2 (P1.17) <-> J9:4 (P1.18)


Then you can run:
```console
$ cargo test
```

And you should see something like this:

```
 Running tests/gpio-input-floating.rs (/Users/allexoll/Desktop/lpc546xx-hal/target/thumbv7em-none-eabihf/debug/deps/gpio_input_floating-4e1c712cdfebaff6)
(HOST) INFO  flashing program (25 pages / 25.00 KiB)
(HOST) INFO  success!
────────────────────────────────────────────────────────────────────────────────
(1/3) running `ground_is_low`...
└─ gpio_input_floating::tests::__defmt_test_entry @ tests/gpio-input-floating.rs:46
(2/3) running `vdd_is_high`...
└─ gpio_input_floating::tests::__defmt_test_entry @ tests/gpio-input-floating.rs:51
(3/3) running `always_passes`...
└─ gpio_input_floating::tests::__defmt_test_entry @ tests/gpio-input-floating.rs:55
all tests passed!
└─ gpio_input_floating::tests::__defmt_test_entry @ tests/gpio-input-floating.rs:25
────────────────────────────────────────────────────────────────────────────────
(HOST) INFO  device halted without error
     Running tests/gpio-input-pulled.rs (/Users/allexoll/Desktop/lpc546xx-hal/target/thumbv7em-none-eabihf/debug/deps/gpio_input_pulled-ff3fa8a4c24f593e)
(HOST) INFO  flashing program (36 pages / 36.00 KiB)
(HOST) INFO  success!
────────────────────────────────────────────────────────────────────────────────
(1/4) running `pulldown_is_low`...
└─ gpio_input_pulled::tests::__defmt_test_entry @ tests/gpio-input-pulled.rs:49
(2/4) running `pulldown_drives_low`...
└─ gpio_input_pulled::tests::__defmt_test_entry @ tests/gpio-input-pulled.rs:61
(3/4) running `pullup_is_high`...
└─ gpio_input_pulled::tests::__defmt_test_entry @ tests/gpio-input-pulled.rs:71
(4/4) running `pullup_drives_high`...
└─ gpio_input_pulled::tests::__defmt_test_entry @ tests/gpio-input-pulled.rs:83
all tests passed!
└─ gpio_input_pulled::tests::__defmt_test_entry @ tests/gpio-input-pulled.rs:24
────────────────────────────────────────────────────────────────────────────────
(HOST) INFO  device halted without error
     Running tests/gpio-output-open-drain.rs (/Users/allexoll/Desktop/lpc546xx-hal/target/thumbv7em-none-eabihf/debug/deps/gpio_output_open_drain-b75e07a3c8a190c5)
(HOST) INFO  flashing program (37 pages / 37.00 KiB)
(HOST) INFO  success!
────────────────────────────────────────────────────────────────────────────────
(1/2) running `set_low_is_low`...
└─ gpio_output_open_drain::tests::__defmt_test_entry @ tests/gpio-output-open-drain.rs:48
(2/2) running `set_high_is_high`...
└─ gpio_output_open_drain::tests::__defmt_test_entry @ tests/gpio-output-open-drain.rs:71
all tests passed!
└─ gpio_output_open_drain::tests::__defmt_test_entry @ tests/gpio-output-open-drain.rs:23
────────────────────────────────────────────────────────────────────────────────
(HOST) INFO  device halted without error
     Running tests/gpio-output-push-pull.rs (/Users/allexoll/Desktop/lpc546xx-hal/target/thumbv7em-none-eabihf/debug/deps/gpio_output_push_pull-92697b0947871cbe)
(HOST) INFO  flashing program (26 pages / 26.00 KiB)
(HOST) INFO  success!
────────────────────────────────────────────────────────────────────────────────
(1/2) running `set_low_is_low`...
└─ gpio_output_push_pull::tests::__defmt_test_entry @ tests/gpio-output-push-pull.rs:48
(2/2) running `set_high_is_high`...
└─ gpio_output_push_pull::tests::__defmt_test_entry @ tests/gpio-output-push-pull.rs:56
all tests passed!
└─ gpio_output_push_pull::tests::__defmt_test_entry @ tests/gpio-output-push-pull.rs:24
────────────────────────────────────────────────────────────────────────────────
(HOST) INFO  device halted without error
```
