# RIO-HW Guidelines

## Public API

## API design rules (naming + structure)

- **When creating a new node, start from the node template.**
  - copy over `_template/template.py` to `submodule/my_module.py`
- **snake_case.py for file names and PascalCase for class names.**
  - `submodule/my_module.py` contains `MyModule` class.
  - avoid fully capitalized acronyms, `UsbCamera` (not `USBCamera`).
- **Prefix-first naming for discoverability (autocomplete).**
  - **Methods**: `add_shape_sphere()` (not `add_sphere_shape()`).
- **Method names are `snake_case`.**
- **Prefer nested classes when self-contained.**
  - If a helper type or an enum is only meaningful inside one parent class and doesn't need a public identity, define it as a nested class instead of creating a new top-level class/module.
- **Follow PEP 8 for Python code.**
- **Use Google-style docstrings.**
  - Write clear, concise docstrings that explain what the function does, its parameters, and its return value.

## Dependencies

- **Avoid adding new required dependencies.** Core should remain lightweight and minimize external requirements. Node-specific dependencies should be isolated within the node's single file implementation.
- **Strongly prefer not adding new optional dependencies.** If additional functionality requires a new package, carefully consider whether the benefit justifies the added complexity and maintenance burden. When possible, implement functionality using existing dependencies, including Warp functions and kernels, NumPy, or the standard library.

## Tooling: prefer `uv` for running, testing, and benchmarking

We standardize on `uv` for local workflows when available. If `uv` is not installed, fall back to a virtual environment created with `venv` or `conda`.

### Run examples

### Run tests

### Pre-commit (lint/format hooks)

```bash
uvx pre-commit run -a
uvx pre-commit install
```

## Commit and Pull Request Guidelines

Follow conventional commit message practices.
- Use clear, descriptive commit messages that explain what changed and why.
- Keep commits focused and atomic—one logical change per commit.
- Reference related issues in commit messages when applicable.

For detailed guidance on writing good commit messages and structuring pull requests, see [Apache Airflow's Pull Request Guidelines](https://github.com/apache/airflow/blob/main/AGENTS.md#pull-request-guidelines).
