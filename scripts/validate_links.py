"""
Link Validator for Physical AI Textbook Documentation
Scans all markdown files for broken relative links and reports issues
"""

import os
import re
from pathlib import Path
from typing import List, Tuple, Dict
from dataclasses import dataclass
import sys

@dataclass
class LinkIssue:
    """Represents a broken link issue"""
    source_file: str
    link_text: str
    target_path: str
    line_number: int
    issue_type: str  # "missing", "invalid", "external"

class LinkValidator:
    """Validates markdown links in documentation"""

    def __init__(self, docs_dir: str = "web/docs"):
        """
        Initialize the link validator

        Args:
            docs_dir: Path to documentation directory
        """
        self.docs_dir = Path(docs_dir)
        self.issues: List[LinkIssue] = []

        # Regex patterns for markdown links
        # Matches [text](path) but not [text](http://...) or [text](https://...)
        self.link_pattern = re.compile(r'\[([^\]]+)\]\(([^)]+)\)')

    def is_external_link(self, link: str) -> bool:
        """Check if link is external (http/https)"""
        return link.startswith(('http://', 'https://', 'mailto:', '#'))

    def resolve_link_path(self, source_file: Path, link: str) -> Path:
        """
        Resolve relative link path from source file

        Args:
            source_file: Source markdown file
            link: Relative link path

        Returns:
            Resolved absolute path
        """
        # Remove anchor fragments
        link = link.split('#')[0]

        # Get directory of source file
        source_dir = source_file.parent

        # Resolve relative path
        target_path = (source_dir / link).resolve()

        return target_path

    def validate_file(self, file_path: Path) -> List[LinkIssue]:
        """
        Validate all links in a markdown file

        Args:
            file_path: Path to markdown file

        Returns:
            List of link issues found
        """
        issues = []

        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                lines = f.readlines()

            for line_num, line in enumerate(lines, start=1):
                # Find all markdown links in the line
                matches = self.link_pattern.finditer(line)

                for match in matches:
                    link_text = match.group(1)
                    link_path = match.group(2)

                    # Skip external links
                    if self.is_external_link(link_path):
                        continue

                    # Resolve the link
                    try:
                        target_path = self.resolve_link_path(file_path, link_path)

                        # Check if target exists
                        if not target_path.exists():
                            issues.append(LinkIssue(
                                source_file=str(file_path.relative_to(self.docs_dir.parent)),
                                link_text=link_text,
                                target_path=link_path,
                                line_number=line_num,
                                issue_type="missing"
                            ))

                    except Exception as e:
                        issues.append(LinkIssue(
                            source_file=str(file_path.relative_to(self.docs_dir.parent)),
                            link_text=link_text,
                            target_path=link_path,
                            line_number=line_num,
                            issue_type=f"invalid: {str(e)}"
                        ))

        except Exception as e:
            print(f"Error reading file {file_path}: {e}")

        return issues

    def scan_directory(self) -> Dict[str, List[LinkIssue]]:
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

        print(f"Scanning {len(markdown_files)} markdown files for broken links...")
        print("-" * 80)

        for md_file in markdown_files:
            issues = self.validate_file(md_file)
            if issues:
                results[str(md_file.relative_to(self.docs_dir.parent))] = issues
                self.issues.extend(issues)

        return results

    def print_report(self, results: Dict[str, List[LinkIssue]]):
        """
        Print a formatted report of link validation results

        Args:
            results: Dictionary of file paths to issues
        """
        if not results:
            print("\n✅ SUCCESS: No broken links found!")
            print(f"   Validated {len(list(self.docs_dir.rglob('*.md')))} markdown files")
            return

        print(f"\n❌ FAILED: Found {len(self.issues)} broken link(s)\n")

        for file_path, issues in results.items():
            print(f"\n📄 {file_path}")
            print("   " + "─" * 76)

            for issue in issues:
                print(f"   Line {issue.line_number}: [{issue.link_text}]({issue.target_path})")
                print(f"   Issue: {issue.issue_type}")
                print()

        print("-" * 80)
        print(f"\n📊 Summary:")
        print(f"   Total files with issues: {len(results)}")
        print(f"   Total broken links: {len(self.issues)}")

        # Group by issue type
        issue_types = {}
        for issue in self.issues:
            issue_types[issue.issue_type] = issue_types.get(issue.issue_type, 0) + 1

        print(f"\n   Breakdown by type:")
        for issue_type, count in sorted(issue_types.items()):
            print(f"     - {issue_type}: {count}")

    def save_report(self, output_file: str = "link_validation_report.txt"):
        """
        Save validation report to file

        Args:
            output_file: Output file path
        """
        with open(output_file, 'w', encoding='utf-8') as f:
            f.write("Link Validation Report\n")
            f.write("=" * 80 + "\n\n")

            if not self.issues:
                f.write("✅ SUCCESS: No broken links found!\n")
            else:
                f.write(f"❌ FAILED: Found {len(self.issues)} broken link(s)\n\n")

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
                        f.write(f"Line {issue.line_number}: [{issue.link_text}]({issue.target_path})\n")
                        f.write(f"Issue: {issue.issue_type}\n\n")

        print(f"\n💾 Report saved to: {output_file}")


def main():
    """Main entry point"""
    import argparse

    parser = argparse.ArgumentParser(
        description="Validate markdown links in documentation"
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
        default="link_validation_report.txt",
        help="Output report file (default: link_validation_report.txt)"
    )
    parser.add_argument(
        "--fail-on-errors",
        action="store_true",
        help="Exit with error code if broken links found"
    )

    args = parser.parse_args()

    # Create validator
    validator = LinkValidator(docs_dir=args.docs_dir)

    # Scan directory
    results = validator.scan_directory()

    # Print report
    validator.print_report(results)

    # Save report
    if results or not results:  # Always save report
        validator.save_report(args.output)

    # Exit with error if requested and issues found
    if args.fail_on_errors and results:
        sys.exit(1)


if __name__ == "__main__":
    main()
