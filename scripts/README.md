# Quality Assurance Scripts

Quality assurance tools for the Physical AI Textbook platform.

## Scripts

### 1. Link Validator (`validate_links.py`)

Scans all markdown files for broken relative links.

**Purpose**: Ensure documentation has no broken internal links before deployment.

**Usage**:
```bash
python scripts/validate_links.py
```

**Options**:
```bash
python scripts/validate_links.py --help

Options:
  --docs-dir PATH        Path to documentation directory (default: web/docs)
  --output PATH          Output report file (default: link_validation_report.txt)
  --fail-on-errors       Exit with error code if broken links found
```

**Example**:
```bash
# Validate links in default directory
python scripts/validate_links.py

# Validate custom directory
python scripts/validate_links.py --docs-dir path/to/docs

# Fail CI if errors found
python scripts/validate_links.py --fail-on-errors
```

**Output**:
```
Scanning 12 markdown files for broken links...
────────────────────────────────────────────────────────────────────────────────

✅ SUCCESS: No broken links found!
   Validated 12 markdown files

💾 Report saved to: link_validation_report.txt
```

**What it checks**:
- ✅ Relative links to other markdown files
- ✅ Links to images, PDFs, and other resources
- ✅ Anchor links within documents
- ❌ Skips external URLs (http://, https://)

---

### 2. Code Linter (`lint_code.py`)

Extracts Python code blocks from markdown and validates syntax.

**Purpose**: Ensure all code examples in documentation are syntactically valid.

**Usage**:
```bash
python scripts/lint_code.py
```

**Options**:
```bash
python scripts/lint_code.py --help

Options:
  --docs-dir PATH        Path to documentation directory (default: web/docs)
  --output PATH          Output report file (default: code_lint_report.txt)
  --fail-on-errors       Exit with error code if syntax errors found
```

**Example**:
```bash
# Lint code blocks in default directory
python scripts/lint_code.py

# Lint custom directory
python scripts/lint_code.py --docs-dir path/to/docs

# Fail CI if errors found
python scripts/lint_code.py --fail-on-errors
```

**Output**:
```
Scanning 12 markdown files for code issues...
────────────────────────────────────────────────────────────────────────────────
Found 15 Python code blocks
────────────────────────────────────────────────────────────────────────────────

✅ SUCCESS: No code syntax errors found!
   Validated Python code blocks in documentation

💾 Report saved to: code_lint_report.txt
```

**What it checks**:
- ✅ Python syntax errors (compile-time)
- ✅ Indentation issues
- ✅ Invalid Python constructs
- ❌ Skips bash/shell commands
- ❌ Skips non-Python code blocks

---

## Integration with CI/CD

### GitHub Actions Example

```yaml
name: Quality Assurance

on:
  push:
    branches: [main]
  pull_request:
    branches: [main]

jobs:
  qa:
    runs-on: ubuntu-latest

    steps:
      - uses: actions/checkout@v3

      - name: Set up Python
        uses: actions/setup-python@v4
        with:
          python-version: '3.11'

      - name: Validate Links
        run: |
          python scripts/validate_links.py --fail-on-errors

      - name: Lint Code Blocks
        run: |
          python scripts/lint_code.py --fail-on-errors

      - name: Upload Reports
        if: failure()
        uses: actions/upload-artifact@v3
        with:
          name: qa-reports
          path: |
            link_validation_report.txt
            code_lint_report.txt
```

---

## Pre-commit Hook

Add to `.git/hooks/pre-commit`:

```bash
#!/bin/bash

echo "Running quality assurance checks..."

# Validate links
python scripts/validate_links.py --fail-on-errors
if [ $? -ne 0 ]; then
  echo "❌ Link validation failed. Fix broken links before committing."
  exit 1
fi

# Lint code blocks
python scripts/lint_code.py --fail-on-errors
if [ $? -ne 0 ]; then
  echo "❌ Code linting failed. Fix syntax errors before committing."
  exit 1
fi

echo "✅ All quality checks passed!"
exit 0
```

Make it executable:
```bash
chmod +x .git/hooks/pre-commit
```

---

## Troubleshooting

### Link Validator

**Issue**: "Documentation directory not found"
**Solution**: Make sure `web/docs/` exists or specify correct path with `--docs-dir`

**Issue**: False positives for external links
**Solution**: Links starting with `http://`, `https://`, or `mailto:` are automatically skipped

**Issue**: Anchor links reported as broken
**Solution**: Anchor fragments are stripped before validation (e.g., `#section` is ignored)

### Code Linter

**Issue**: Bash commands reported as syntax errors
**Solution**: Use proper code block markers:
- ✅ ` ```bash` or ` ```shell` for shell commands
- ✅ ` ```python` or ` ```py` for Python code

**Issue**: Incomplete code snippets fail
**Solution**: Code snippets must be syntactically complete. Use `# ...` for omitted code.

**Example**:
```python
# ✅ Good
def my_function():
    # ... implementation
    pass

# ❌ Bad (syntax error)
def my_function():
    # ... implementation
```

---

## Development

### Running Locally

```bash
# Clone repository
git clone <repo-url>
cd Hackathon_1

# Run validators
python scripts/validate_links.py
python scripts/lint_code.py
```

### Testing

```bash
# Test with sample markdown
mkdir -p test_docs
echo "[Broken Link](./nonexistent.md)" > test_docs/test.md
python scripts/validate_links.py --docs-dir test_docs
```

---

## Dependencies

**Link Validator**:
- Python 3.8+
- Standard library only (no external dependencies)

**Code Linter**:
- Python 3.8+
- Standard library only (no external dependencies)

---

## Output Reports

Both scripts generate detailed reports saved to text files.

### Link Validation Report

```
Link Validation Report
================================================================================

❌ FAILED: Found 2 broken link(s)

web/docs/module-1-nervous-system/intro.md
--------------------------------------------------------------------------------
Line 45: [ROS 2 Architecture](./ros2-architecture.md)
Issue: missing

Line 52: [Creating Your First Node](./first-node.md)
Issue: missing
```

### Code Lint Report

```
Code Lint Report
================================================================================

❌ FAILED: Found 1 code issue(s)

web/docs/module-1-nervous-system/intro.md
--------------------------------------------------------------------------------

Code Block #3
Line 5: SyntaxError: invalid syntax

Code snippet:
  def my_function()
    print("Hello")
```

---

## Best Practices

1. **Run validators before committing**: Catch issues early
2. **Fix issues immediately**: Don't accumulate technical debt
3. **Use in CI/CD**: Prevent broken links from reaching production
4. **Regular audits**: Run weekly to catch new issues
5. **Document exceptions**: If a link should be broken (404 tutorial), document why

---

## Future Enhancements

- [ ] Check external URLs for 404s (rate-limited)
- [ ] Validate image files exist and are under size limit
- [ ] Check for duplicate headings (SEO issue)
- [ ] Validate frontmatter/metadata
- [ ] Spell check content
- [ ] Check code style (beyond syntax)
- [ ] Validate API endpoint references

---

## License

Part of the Physical AI & Humanoid Robotics Textbook project.
