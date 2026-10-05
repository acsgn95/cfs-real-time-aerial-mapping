# Contributing

Thank you for your interest in contributing to cFS Real-Time Aerial Mapping.

This repository supports ongoing master's thesis research on real-time photogrammetric mapping from simulated UAV data using NASA's Core Flight System. The architecture and interfaces are still evolving, so proposed changes should remain aligned with the documented research scope.

## Ways to Contribute

Contributions may include:

- Bug reports
- Documentation improvements
- Test cases
- Build and CI improvements
- cFS application development
- Coordinate-frame and georeferencing utilities
- Simulation integration
- Performance instrumentation
- Reproducible experiment configurations

Before implementing a substantial change, please open an issue to discuss its scope and intended design.

## Development Workflow

1. Fork the repository.
2. Create a branch from `main`.
3. Make focused changes.
4. Add or update relevant tests and documentation.
5. Run the pre-commit checks.
6. Open a pull request.

Recommended branch names include:

```text
feature/short-description
fix/short-description
docs/short-description
test/short-description
```

## Development Environment

Install pre-commit:

```bash
python -m pip install pre-commit
pre-commit install
```

Run all configured checks:

```bash
pre-commit run --all-files
```

All changes must pass the repository's automated checks before they can be merged.

Build and test instructions will be added after the initial CMake and cFS development environment is established.

## Coding Guidelines

Contributions should:

- Follow the formatting rules defined in `.editorconfig`
- Use LF line endings unless otherwise specified in `.gitattributes`
- Avoid committing generated files, build products, large datasets, or credentials
- Keep changes focused and reasonably small
- Include clear comments where behavior is not self-evident
- Preserve existing third-party copyright and license notices
- Document coordinate frames, units, timestamps, and transformation conventions explicitly
- Use SI units unless a documented interface requires otherwise

C and C++ formatting rules will be documented and enforced after the project's coding standard is finalized.

## Commit Messages

Use concise commit messages written in the imperative form.

The following prefixes are recommended:

```text
feat: add a new capability
fix: correct defective behavior
docs: update documentation
test: add or update tests
ci: modify continuous integration
build: modify the build system
refactor: restructure code without changing behavior
chore: perform repository maintenance
```

Example:

```text
feat: add camera pose message definition
```

## Pull Requests

A pull request should:

- Describe the problem and proposed solution
- Reference the related issue when applicable
- Explain any architectural or interface changes
- Include tests or justify why tests are not required
- Update relevant documentation
- Pass all automated checks
- Avoid unrelated formatting or refactoring changes

Draft pull requests are welcome for early design feedback.

## Tests

New functionality should include appropriate automated tests whenever practical.

Tests should be deterministic and should not depend on unavailable external services or large private datasets. Small synthetic or redistributable test data may be stored under `sample_data/`.

## Data and Generated Outputs

Do not commit:

- Large image datasets
- Generated dense point clouds
- DSM or orthomosaic outputs
- COLMAP databases or workspaces
- Simulator cache files
- Credentials, tokens, or private configuration
- Data without clear redistribution rights

Dataset download instructions, checksums, and licenses should be documented separately.

## Third-Party Software

Third-party software remains subject to its original license and copyright terms.

Do not copy external source code into this repository unless its license is compatible and the required attribution and license notices are preserved.

## Research Scope

This project focuses on modular real-time photogrammetric mapping using known camera poses supplied by a simulation environment.

Visual localization, SLAM-based trajectory estimation, operational flight control, and safety-critical navigation are outside the current research scope unless explicitly introduced through an approved design change.

## License

By submitting a contribution, you agree that your contribution will be licensed under the Apache License 2.0, unless explicitly stated otherwise for an approved third-party component.

## Questions

For questions, design discussions, or feature proposals, please open a GitHub issue.
