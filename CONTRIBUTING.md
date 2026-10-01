# Contributing to ax52

Thanks for your interest. Bug reports, hardware test reports and patches are all
welcome.

## Scope

ax52 is a station-mode driver for the RTL8852BE. Fixes, hardware coverage and the
features listed as "not implemented" in the [README](README.md) are in scope. For
larger additions (AP or P2P mode, power saving, firmware scan offload), please open
an issue first so the design can be agreed before you write code.

## How the code is written

Read [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) first; it maps the driver and
its locking rules.

The project is **specification first**. Hardware behaviour (register sequences,
values, table formats, firmware command layouts) is written down in
[`docs/spec/`](docs/spec/) before it is implemented:

- Resolve it for RTL8852B (PCIe, the chip cut and RFE type in question). Give
  numeric registers and masks, and cite the rtw89 sources it was checked against
  as `file:line` in the tree named in the README.
- Implement it in ax52's own structure. Hardware facts are shared with rtw89 (which
  is `GPL-2.0 OR BSD-3-Clause`), but do not paste rtw89 code; describe the behaviour
  and write the driver code fresh.
- If the code deliberately differs from rtw89, say so in a comment and explain why.

## Style and checks

- Linux kernel coding style. New files start with an SPDX line:
  `// SPDX-License-Identifier: GPL-2.0` for `.c`, `/* … */` for headers.
- Match the surrounding code: short comments that explain *why*, named constants
  for registers in the file or header that owns them.
- `make check` must pass (`W=1 -Werror`, sparse, checkpatch, shellcheck and
  firmware-parser lint). CI runs the same target on every push and pull request.

## Commits

- One logical change per commit, and it should build.
- Subject: `<area>: <imperative summary>`, where the area is the subsystem, for
  example `phy: add CFO tracking` or `docs: describe the TX path`.
- In the body, explain what and why. For hardware changes, also say what it was
  tested on (chip cut, RFE type, band and width).
- Sign off your commits (`git commit -s`). The `Signed-off-by` line certifies the
  [Developer Certificate of Origin](https://developercertificate.org/), as in the
  Linux kernel.

## Testing

Follow [docs/TESTING.md](docs/TESTING.md). In the pull request, state which
hardware steps you ran, or say that the change is untested on hardware. A
documentation or tooling change is fine to submit without hardware.

## Reporting problems

- **Bugs**: use the bug report form. It asks for the kernel log lines and the
  `tools/check.sh` output needed to debug a driver remotely.
- **Hardware results**: use the hardware test report form, especially for chip
  cuts or RFE types other than cut B / RFE 1.
- **Security issues**: see [SECURITY.md](SECURITY.md); please do not open a public
  issue.
