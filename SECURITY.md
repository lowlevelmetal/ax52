# Security policy

ax52 runs in the kernel. It parses data from the device (RX descriptors, firmware
events) and from the firmware file, so a memory-safety bug in any of those paths is
a security bug.

## Reporting

Please report vulnerabilities privately through GitHub's
[private vulnerability reporting](https://github.com/lowlevelmetal/ax52/security/advisories/new)
rather than in a public issue. Include the ax52 version (`modinfo -F version
src/ax52.ko`), the kernel version and how to trigger the problem.

## Supported versions

Only the current `main` branch receives fixes.
