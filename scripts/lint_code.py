"""
Code Block Linter for Physical AI Textbook Documentation
Extracts Python code blocks from markdown and validates syntax
"""

import os
import re
import sys
import tempfile
import subprocess
from pathlib import Path
from typing import List, Dict, Tuple
from dataclasses import dataclass

@dataclass
class CodeIssue:
    """Represents a code validation issue"""
    source_file: str
    block_number: int
    line_in_block: int
    error_message: str
    code_snippet: str

class CodeLinter:
    """Lints Python code blocks in markdown documentation"""

    def __init__(self, docs_dir: str = "web/docs"):
        """
        Initialize the code linter

        Args:
            docs_dir: Path to documentation directory
        """
        self.docs_dir = Path(docs_dir)
        self.issues: List[CodeIssue] = []

        # Regex pattern for Python code blocks
        # Matches ```python or ```py or ``` (generic)
        self.code_block_pattern = re.compile(
            r'```(?:python|py)?\n(.*?)```',
            re.DOTALL
        )

    def extract_code_blocks(self, file_path: Path) -> List[Tuple[int, str]]:
        """
        Extract all Python code blocks from a markdown file

        Args:
            file_path: Path to markdown file

        Returns:
            List of tuples (block_number, code_content)
        """
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            blocks = []
            matches = self.code_block_pattern.finditer(content)

            for i, match in enumerate(matches, start=1):
                code = match.group(1).strip()

                # Skip empty blocks
                if not code:
                    continue

                # Skip non-Python code (simple heuristic)
                # If block contains bash/shell commands, skip
                if any(code.strip().startswith(cmd) for cmd in ['$', '#', 'npm', 'curl', 'docker']):
                    continue

                blocks.append((i, code))

            return blocks

        except Exception as e:
            print(f"Error reading file {file_path}: {e}")
            return []

    def validate_code_block(
        self,
        file_path: Path,
        block_number: int,
        code: str
    ) -> List[CodeIssue]:
        """
        Validate a Python code block for syntax errors

        Args:
            file_path: Source markdown file
            block_number: Block number in the file
            code: Python code to validate

        Returns:
            List of code issues found
        """
        issues = []

        try:
            # First, try simple compilation
            compile(code, '<string>', 'exec')

        except SyntaxError as e:
            # Extract relevant error information
            error_msg = f"SyntaxError: {e.msg}"
            if e.lineno:
                error_msg += f" (line {e.lineno})"

            # Get code snippet around error
            lines = code.split('\n')
            snippet_start = max(0, (e.lineno or 1) - 2)
            snippet_end = min(len(lines), (e.lineno or 1) + 2)
            snippet = '\n'.join(lines[snippet_start:snippet_end])

            issues.append(CodeIssue(
                source_file=str(file_path.relative_to(self.docs_dir.parent)),
                block_number=block_number,
                line_in_block=e.lineno or 0,
                error_message=error_msg,
                code_snippet=snippet
            ))

        except Exception as e:
            # Other compilation errors
            issues.append(CodeIssue(
                source_file=str(file_path.relative_to(self.docs_dir.parent)),
                block_number=block_number,
                line_in_block=0,
                error_message=f"Error: {str(e)}",
                code_snippet=code[:200] + "..." if len(code) > 200 else code
            ))

        return issues

    def lint_file(self, file_path: Path) -> List[CodeIssue]:
        """
        Lint all Python code blocks in a markdown file

        Args:
            file_path: Path to markdown file

        Returns:
            List of code issues found
        """
        issues = []

        # Extract code blocks
        code_blocks = self.extract_code_blocks(file_path)

        # Validate each block
        for block_number, code in code_blocks:
            block_issues = self.validate_code_block(file_path, block_number, code)
            issues.extend(block_issues)

        return issues

    def scan_directory(self) -> Dict[str, List[CodeIssue]]:
        """
        Scan all markdown files in the docs directory

        Returns:
            Dictionary mapping file paths to lists of issues
        """
        if not self.docs_dir.exists():
            print(f"Error: Documentation directory not found: {self.docs_dir}")
            return {}

        results = {}
        markdown_files = list(self.docs_dir.rglob("*.md"))

        print(f"Scanning {len(markdown_files)} markdown files for code issues...")
        print("-" * 80)

        total_blocks = 0

        for md_file in markdown_files:
            code_blocks = self.extract_code_blocks(md_file)
            total_blocks += len(code_blocks)

            issues = self.lint_file(md_file)
            if issues:
                results[str(md_file.relative_to(self.docs_dir.parent))] = issues
                self.issues.extend(issues)

        print(f"Found {total_blocks} Python code blocks")
        print("-" * 80)

        return results

    def print_report(self, results: Dict[str, List[CodeIssue]]):
        """
        Print a formatted report of code validation results

        Args:
            results: Dictionary of file paths to issues
        """
        if not results:
            print("\n✅ SUCCESS: No code syntax errors found!")
            print(f"   Validated Python code blocks in documentation")
            return

        print(f"\n❌ FAILED: Found {len(self.issues)} code issue(s)\n")

        for file_path, issues in results.items():
            print(f"\n📄 {file_path}")
            print("   " + "─" * 76)

            for issue in issues:
                print(f"   Code Block #{issue.block_number}")
                if issue.line_in_block > 0:
                    print(f"   Line {issue.line_in_block} in block: {issue.error_message}")
                else:
                    print(f"   {issue.error_message}")

                print(f"\n   Code snippet:")
                for line in issue.code_snippet.split('\n'):
                    print(f"      {line}")
                print()

        print("-" * 80)
        print(f"\n📊 Summary:")
        print(f"   Total files with issues: {len(results)}")
        print(f"   Total code errors: {len(self.issues)}")

    def save_report(self, output_file: str = "code_lint_report.txt"):
        """
        Save validation report to file

        Args:
            output_file: Output file path
        """
        with open(output_file, 'w', encoding='utf-8') as f:
            f.write("Code Lint Report\n")
            f.write("=" * 80 + "\n\n")

            if not self.issues:
                f.write("✅ SUCCESS: No code syntax errors found!\n")
            else:
                f.write(f"❌ FAILED: Found {len(self.issues)} code issue(s)\n\n")

                # Group by file
                by_file = {}
                for issue in self.issues:
                    if issue.source_file not in by_file:
                        by_file[issue.source_file] = []
                    by_file[issue.source_file].append(issue)

                for file_path, issues in sorted(by_file.items()):
                    f.write(f"\n{file_path}\n")
                    f.write("-" * 80 + "\n")

                    for issue in issues:
                        f.write(f"\nCode Block #{issue.block_number}\n")
                        if issue.line_in_block > 0:
                            f.write(f"Line {issue.line_in_block}: {issue.error_message}\n")
                        else:
                            f.write(f"{issue.error_message}\n")

                        f.write(f"\nCode snippet:\n")
                        for line in issue.code_snippet.split('\n'):
                            f.write(f"  {line}\n")
                        f.write("\n")

        print(f"\n💾 Report saved to: {output_file}")


def main():
    """Main entry point"""
    import argparse

    parser = argparse.ArgumentParser(
        description="Lint Python code blocks in markdown documentation"
    )
    parser.add_argument(
        "--docs-dir",
        type=str,
        default="web/docs",
        help="Path to documentation directory (default: web/docs)"
    )
    parser.add_argument(
        "--output",
        type=str,
        default="code_lint_report.txt",
        help="Output report file (default: code_lint_report.txt)"
    )
    parser.add_argument(
        "--fail-on-errors",
        action="store_true",
        help="Exit with error code if syntax errors found"
    )

    args = parser.parse_args()

    # Create linter
    linter = CodeLinter(docs_dir=args.docs_dir)

    # Scan directory
    results = linter.scan_directory()

    # Print report
    linter.print_report(results)

    # Save report
    if results or not results:  # Always save report
        linter.save_report(args.output)

    # Exit with error if requested and issues found
    if args.fail_on_errors and results:
        sys.exit(1)


if __name__ == "__main__":
    main()
