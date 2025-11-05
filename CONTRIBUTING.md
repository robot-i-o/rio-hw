# Contributing

1. Fork the repository and develop features on a branch with the following naming scheme: `$USER/feature-name`.

2. Code formatting and linting with [Ruff](https://docs.astral.sh/ruff/).
- Option 1: Use `uvx ruff check` and `uvx format --check` to check for code quality issues.
- Option 2: Use `uvx pre-commit run -a`. To automatically run pre-commit hooks with `git commit`, run `uvx pre-commit install`.

3. Spell checking with [Typos](https://github.com/crate-ci/typos).
- Option 1: Use `uvx typos --diff` and `uvx typos -w` to check for spelling errors.
- Option 2: Use `pre-commit` hook.
- **File exclusions:** Use `files.extend-include` under `[tool.typos]` section to ignore matching files and directories.
- **Word allowlist:** Use `[tool.typos.default.extend-words]` to allow words or acronyms that would otherwise be flagged.
- **Identifier allowlist:** Use `[tool.typos.default.extend-identifiers]` to allow variables or constants that would otherwise be flagged.

4. Run tests with `uv run --extra dev pytest -n 4`.

5. Open a pull request on the main repository.

### Style Guide
- Follow PEP 8 as the baseline for coding style, but prioritize matching the existing style and conventions of the file being modified to maintain consistency.
- Use [Google-style docstrings](https://google.github.io/styleguide/pyguide.html#38-comments-and-docstrings) (compatible with Napoleon extension for Sphinx docs).
- Write clear, concise commit messages.
- Keep pull requests focused on a single feature or bug fix.
- Aim for consistency in variable and function names.
- Use `TODO(team): COMMENT` to mark general TODOs and `TODO($USER): COMMENT` to mark user-specific TODOs.
