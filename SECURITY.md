# Security Policy

## Supported Versions

| Version | Supported |
|---------|-----------|
| 1.0.x   | Yes       |
| < 1.0   | No        |

## Scope

Raasta is a navigation/pathfinding library that processes geometric data (polygons, grids, positions). Security-relevant concerns include:

- **Denial of service** — pathologically large inputs causing excessive memory/CPU usage
- **Integer overflow** — grid dimension calculations
- **Stack overflow** — recursive algorithms on large inputs (mitigated: all recursion is iterative)

Raasta does not process untrusted network input, handle authentication, or manage secrets.

## Reporting a Vulnerability

Please report security vulnerabilities privately via GitHub's security advisory feature.

Do not open public issues for security vulnerabilities.
