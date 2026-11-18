# Log format v0

A log is a UTF-8 text file that holds ground truth, measurements and tracks for one run. The C++
functions `write_log` and `read_log` in `core/include/kite/log.hpp` read and write it. The reader
accepts exactly what the writer produces.

## Layout

- Line 1 is exactly `# kite-log v0`.
- Every following line is one record. Fields are separated by a comma. There are no spaces.
- Every line ends with `\n`, on every platform. Files are opened in binary mode, so Windows does not
  turn `\n` into `\r\n`. A `\r` at the end of a line is an error.
- The writer puts all `gt` lines first, then all `meas` lines, then all `track` lines, each group in
  the order of its list. The reader accepts the three types in any order and keeps the file order
  within each type.

## Records

| Record | Fields |
|---|---|
| ground truth | `gt,t,ego_id,actor_id,x,y,vx,vy` |
| measurement | `meas,t,ego_id,sensor_id,x,y` |
| track | `track,t,ego_id,track_id,x,y,vx,vy` |

| Field | Type | Meaning |
|---|---|---|
| `t` | double | Time in seconds. |
| `ego_id` | int | The driving car this record belongs to. It is always 0 for now. The format carries it so several driving cars can be added later. The reader and writer do not enforce 0. |
| `actor_id` | int | The true identity of a simulated actor. Only `gt` records have it. |
| `sensor_id` | int | The sensor that produced the measurement. |
| `track_id` | int | The identity the tracker gave to a track. It has no link to `actor_id`. |
| `x`, `y` | double | Position in meters. |
| `vx`, `vy` | double | Velocity in meters per second. |

## Measurements never carry a true identity

A `meas` record has no actor field and no other true identity. A tracker sees only measurements, so
it cannot read the answer from the log. Ground truth is for scoring only. Any new field in a
`meas` record must not reveal which actor produced it.

## Number format

- Doubles are written with `std::to_chars` in its shortest round-trip form, so reading a value back
  gives the same bits. Examples: `0.1`, `-3.25`, `8`, `1e+300`, `-1e-300`.
- Ints are written with `std::to_chars`.
- Values are parsed with `std::from_chars`. The whole field must be consumed.
- Number formatting never uses `printf`, streams or the locale, so the output is the same on every
  machine.
- Every double must be finite. `nan` and `inf` are not valid in a file.

## Errors

The reader throws `std::runtime_error` for each case below. The message names the file line. Lines
count from 1 and the header is line 1. Example: `line 4: unknown record type 'foo'`.

- The header is missing, which includes an empty file, or is not exactly `# kite-log v0`.
- A record type is not `gt`, `meas` or `track`. An empty line has an empty type, so it is an error.
- A record has the wrong number of fields.
- A field is not a number, has trailing characters, or is out of range for its type. A leading `+`
  and any space are errors.
- A double is not finite: `nan`, `inf` or `-inf`.

A file that cannot be opened also throws `std::runtime_error`.

The writer throws `std::runtime_error` on a non-finite value, so every file it writes is readable.
It builds the whole text first, so a failed write leaves no new file behind.

## Example

```
# kite-log v0
gt,0,0,1,12.5,-3.25,8,0
gt,0.1,0,1,13.3,-3.25,8,0
meas,0.1,0,0,13.31,-3.2
track,0.1,0,1,13.3,-3.25,7.9,0.05
```

The `WritesExactBytesForTinyLog` test in `core/tests/log_test.cpp` writes these four records and
compares the file with this text byte for byte.

## Changing the format

Any change to the format bumps the version line and updates this file in the same PR.
