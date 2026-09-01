#!/usr/bin/env python3

"""Add and audit learning annotations across CARKit and vendored source.

The annotator is intentionally conservative: it never replaces an existing
comment or docstring, and it ignores generated artifacts. Python is parsed with
the standard AST. C, C++, JavaScript, and shell functions use guarded
definition matching, so the script also reports anything it cannot recognize.
"""

from __future__ import annotations

import argparse
import ast
from dataclasses import dataclass
from pathlib import Path
import re
import textwrap


ROOTS = ("carkit", "docker", "desktop", "tools")
SKIPPED_PARTS = {
    ".git",
    ".mypy_cache",
    ".pytest_cache",
    "__pycache__",
    "build",
    "install",
    "log",
    "node_modules",
}
CODE_SUFFIXES = {
    ".bash",
    ".c",
    ".cc",
    ".cpp",
    ".cxx",
    ".css",
    ".h",
    ".hh",
    ".hpp",
    ".html",
    ".js",
    ".py",
    ".sh",
}
CONFIG_SUFFIXES = {
    ".action",
    ".cfg",
    ".cmake",
    ".msg",
    ".repos",
    ".rules",
    ".rviz",
    ".srv",
    ".xacro",
    ".xml",
    ".yaml",
    ".yml",
}
MARKER = "CARKit learning annotation:"
PROTECTED_FILES = {
    Path("carkit/vehicle/osracer/osracer_base/launch/description.launch.py"),
}


@dataclass
class Result:
    """Record annotation and audit counts for one run."""

    files_seen: int = 0
    files_changed: int = 0
    functions_seen: int = 0
    functions_added: int = 0
    functions_missing: int = 0


def words(name: str) -> str:
    """Convert a source identifier into readable lowercase words."""
    value = re.sub(r"([a-z0-9])([A-Z])", r"\1 \2", name)
    return value.strip("_").replace("_", " ").lower()


def function_description(name: str) -> str:
    """Create a concise purpose sentence from a conventional function name."""
    special = {
        "__init__": "Initialize the object and the state it owns.",
        "__enter__": "Enter the managed context and return its active value.",
        "__exit__": "Leave the managed context and release its resources.",
        "__call__": "Invoke this object as a callable.",
        "__eq__": "Compare this object with another value for equality.",
        "__ne__": "Compare this object with another value for inequality.",
        "__len__": "Return the number of values represented by this object.",
        "__iter__": "Return an iterator over the values in this object.",
        "__next__": "Return the next value from this iterator.",
        "__str__": "Return the human-readable representation of this object.",
        "__repr__": "Return the diagnostic representation of this object.",
        "main": "Run the command-line entry point and perform clean shutdown.",
        "_main": "Initialize ROS and run the selected node until shutdown.",
        "clamp": "Limit a numeric value to the inclusive lower and upper bounds.",
        "_parameter": "Return the current value of a declared ROS parameter.",
        "_configuration": "Collect current parameters into an algorithm configuration.",
        "_declare_parameters": "Declare the ROS parameters and their safe default values.",
        "_update": "Run one periodic update and publish the resulting output.",
        "guided_main": "Run the guided student implementation entry point.",
        "boilerplate_main": "Run the safe boilerplate student implementation entry point.",
        "profiles": "Load course profile documents keyed by profile name.",
        "yaw_from_quaternion": "Convert a quaternion orientation into planar yaw radians.",
        "inference_due": "Return whether enough time has elapsed to run another inference.",
        "republishMap": "Request the bridge's retained occupancy map for immediate display.",
        "socketSend": "Send one JSON operation through the active ROS WebSocket.",
        "scheduleDraw": "Coalesce visualization updates into the next animation frame.",
        "resetView": "Restore the map camera to its default zoom, rotation, and pan.",
        "generate_launch_description": (
            "Build and return the ROS 2 launch description for this package."
        ),
    }
    if name in special:
        return special[name]
    readable = words(name)
    prefixes = (
        ("test ", "Verify that "),
        ("on ", "Handle "),
        ("handle ", "Handle "),
        ("parse ", "Parse and validate "),
        ("read ", "Read and return "),
        ("load ", "Load and return "),
        ("get ", "Return "),
        ("find ", "Find and return "),
        ("compute ", "Compute and return "),
        ("calculate ", "Calculate and return "),
        ("create ", "Create and return "),
        ("make ", "Create and return "),
        ("build ", "Build and return "),
        ("publish ", "Publish "),
        ("send ", "Send "),
        ("write ", "Write "),
        ("update ", "Update "),
        ("set ", "Set "),
        ("configure ", "Configure "),
        ("validate ", "Validate "),
        ("check ", "Check "),
        ("is ", "Return whether "),
        ("has ", "Return whether "),
        ("can ", "Return whether "),
        ("should ", "Return whether "),
        ("enable ", "Enable "),
        ("disable ", "Disable "),
        ("add ", "Add "),
        ("remove ", "Remove "),
        ("delete ", "Delete "),
        ("process ", "Process "),
        ("transform ", "Transform "),
        ("convert ", "Convert "),
        ("encode ", "Encode "),
        ("decode ", "Decode "),
        ("serialize ", "Serialize "),
        ("deserialize ", "Deserialize "),
        ("normalize ", "Normalize "),
        ("initialize ", "Initialize "),
        ("init ", "Initialize "),
        ("open ", "Open "),
        ("connect ", "Connect "),
        ("disconnect ", "Disconnect "),
        ("accept ", "Accept "),
        ("close ", "Close "),
        ("stop ", "Stop "),
        ("start ", "Start "),
        ("reset ", "Reset "),
        ("run ", "Run "),
    )
    for prefix, verb in prefixes:
        if readable.startswith(prefix):
            return f"{verb}{readable[len(prefix):]}."
    if "callback" in readable:
        return f"Handle {readable.replace('callback', '').strip()} messages."
    if readable.endswith(" json"):
        return f"Convert {readable[:-5]} data into a JSON object for browser transport."
    if readable.startswith("operator"):
        return f"Implement the C++ {readable} overload for this type."
    return f"Perform the {readable} operation."


def generated_descriptions(name: str) -> set[str]:
    """Return current and legacy descriptions emitted by this annotator."""
    descriptions = {function_description(name)}
    if name[:1].isupper():
        descriptions.add(f"Initialize a {words(name)} instance and its owned state.")
    return descriptions


def looks_generated_description(name: str, description: str) -> bool:
    """Recognize annotations generated by this tool without touching authored prose."""
    return bool(
        description in generated_descriptions(name)
        or (
            description.startswith("Perform the ")
            and description.endswith(" operation.")
        )
        or (
            description.startswith("Initialize a ")
            and description.endswith(" instance and its owned state.")
        )
    )


EXPLANATION_WORTHY_FUNCTIONS = {
    "_configuration",
    "_declare_parameters",
    "_update",
    "boilerplate_main",
    "generate_launch_description",
    "guided_main",
    "inference_due",
    "profiles",
    "republishMap",
    "resetView",
    "scheduleDraw",
    "socketSend",
    "yaw_from_quaternion",
}


def should_annotate_function(name: str, description: str) -> bool:
    """Keep generated prose only when it adds information beyond the identifier."""
    return bool(
        name in EXPLANATION_WORTHY_FUNCTIONS
        or "browser transport" in description
        or "ROS parameter" in description
        or "algorithm configuration" in description
    )


def annotation_for(path: Path) -> str:
    """Describe a file's teaching role using its path and extension."""
    name = path.name
    package = path.parent.name
    if name == "CMakeLists.txt":
        return f"defines build, dependency, and install rules for {package}."
    if name == "package.xml":
        return f"declares ROS metadata and dependencies for {package}."
    if name == "setup.cfg":
        return f"configures Python installation and tooling for {package}."
    if name.startswith("Dockerfile"):
        return "builds the reproducible CARKit container environment."
    if "launch" in path.parts or name.endswith(".launch.py"):
        return "assembles ROS nodes, parameters, and remappings for startup."
    if "behavior_trees" in path.parts:
        return "defines the Nav2 behavior-tree control flow used at runtime."
    if "test" in path.parts or name.startswith("test_"):
        return "verifies a runtime contract without changing production state."
    if "config" in path.parts or path.suffix in CONFIG_SUFFIXES:
        return "documents runtime settings consumed by the surrounding package."
    if path.suffix in {".h", ".hh", ".hpp"}:
        return "declares interfaces implemented by the corresponding source."
    if path.suffix in {".sh", ".bash"}:
        return "orchestrates a repeatable CARKit command-line workflow."
    return "implements the behavior described by this file's package and module."


def is_test_path(path: Path) -> bool:
    """Return whether a path contains tests or test-only fixtures."""
    lowered_parts = {part.lower().replace("_", "-") for part in path.parts}
    lowered_name = path.name.lower()
    return bool(
        lowered_parts.intersection({"test", "tests", "unit-tests", "test-data"})
        or lowered_name.startswith("test_")
        or lowered_name == "conftest.py"
        or lowered_name.endswith(("_test.py", ".test.py", ".test"))
    )


def supported_source(path: Path) -> bool:
    """Return whether a non-generated file supports safe source comments."""
    name = path.name
    return bool(
        path.suffix.lower() in CODE_SUFFIXES | CONFIG_SUFFIXES
        or name == "CMakeLists.txt"
        or name.startswith("Dockerfile")
    )


def source_files(root: Path) -> list[Path]:
    """Return supported, non-generated source files in deterministic order."""
    files = []
    for relative_root in ROOTS:
        directory = root / relative_root
        if not directory.exists():
            continue
        for path in directory.rglob("*"):
            relative_path = path.relative_to(root)
            if (
                not path.is_file()
                or any(part in SKIPPED_PARTS for part in path.parts)
                or is_test_path(relative_path)
                or relative_path in PROTECTED_FILES
            ):
                continue
            if supported_source(path):
                files.append(path)
    return sorted(files)


def read_source(path: Path) -> str:
    """Read text without normalizing vendor line endings."""
    with path.open("r", encoding="utf-8", errors="surrogateescape", newline="") as stream:
        return stream.read()


def write_source(path: Path, text: str) -> None:
    """Write text while preserving line endings already present in the content."""
    with path.open("w", encoding="utf-8", errors="surrogateescape", newline="") as stream:
        stream.write(text)


def comment_insertion_line(lines: list[str]) -> int:
    """Find a safe top-of-file location after shebang and license comments."""
    index = 0
    if lines and lines[0].startswith("#!"):
        index = 1
    while index < len(lines) and not lines[index].strip():
        index += 1
    if index < len(lines) and lines[index].lstrip().startswith(("#", "//", "/*", "*")):
        while index < len(lines):
            stripped = lines[index].lstrip()
            if stripped.startswith(("#", "//", "/*", "*")) or not stripped.strip():
                index += 1
                continue
            break
    return index


def cpp_comment_insertion_line(lines: list[str]) -> int:
    """Find a C/C++ header location without treating preprocessor lines as comments."""
    index = 0
    while index < len(lines) and not lines[index].strip():
        index += 1
    if index < len(lines) and lines[index].lstrip().startswith(("//", "/*", "*")):
        while index < len(lines):
            stripped = lines[index].lstrip()
            if stripped.startswith(("//", "/*", "*")) or not stripped.strip():
                index += 1
                continue
            break
    return index


def wrap_docstring(description: str, indent: str) -> list[str]:
    """Format a PEP 257 docstring without exceeding normal line limits."""
    width = max(36, 88 - len(indent) - 6)
    parts = textwrap.wrap(description, width=width)
    if len(parts) == 1:
        return [f'{indent}"""{parts[0]}"""\n']
    output = [f'{indent}"""{parts[0]}\n']
    output.extend(f"{indent}{part}\n" for part in parts[1:])
    output[-1] = output[-1].rstrip("\n") + '"""\n'
    return output


def annotate_python(path: Path, text: str, result: Result) -> str:
    """Insert missing Python function docstrings using AST source locations."""
    tree = ast.parse(text, filename=str(path))
    lines = text.splitlines(keepends=True)
    edits: list[tuple[int, int, list[str]]] = []
    for node in ast.walk(tree):
        if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            continue
        result.functions_seen += 1
        existing = ast.get_docstring(node)
        if existing is not None:
            description = function_description(node.name)
            statement = node.body[0]
            if (
                existing.startswith("Perform the ")
                and existing.endswith(" operation.")
                and description != existing
                and statement.lineno == statement.end_lineno
            ):
                indent = " " * (node.col_offset + 4)
                edits.append(
                    (
                        statement.lineno - 1,
                        statement.end_lineno,
                        wrap_docstring(description, indent),
                    )
                )
            continue
        if not node.body or node.body[0].lineno <= node.lineno:
            result.functions_missing += 1
            continue
        description = function_description(node.name)
        if not should_annotate_function(node.name, description):
            # A comment that merely repeats a function name adds noise.
            continue
        indent = " " * (node.col_offset + 4)
        edits.append(
            (
                node.body[0].lineno - 1,
                node.body[0].lineno - 1,
                wrap_docstring(description, indent),
            )
        )
    # AST locations refer to the original text. Applying every edit from the
    # bottom upward prevents a multi-line docstring from shifting a later edit.
    for start, end, content in sorted(edits, key=lambda edit: edit[0], reverse=True):
        lines[start:end] = content
    result.functions_added += sum(start == end for start, end, _ in edits)
    output = "".join(lines)
    if MARKER not in output:
        updated = output.splitlines(keepends=True)
        index = comment_insertion_line(updated)
        updated[index:index] = [f"# {MARKER} {annotation_for(path)}\n"]
        output = "".join(updated)
    return output


CPP_CONTROL_WORDS = {"if", "for", "while", "switch", "catch", "return", "sizeof"}
CPP_DEFINITION = re.compile(
    r"(?P<name>(?:[A-Za-z_]\w*::)*(?:operator\s*[^\s(]+|~?[A-Za-z_]\w*))\s*\([^;{}]*\)"
    r"(?:\s*(?:const|noexcept|override|final|->\s*[^\s{]+))*"
    r"(?:\s*:\s*[^{}]+)?\s*\{\s*$"
)


def previous_code_line(lines: list[str], index: int) -> str:
    """Return the closest non-empty line before an insertion point."""
    for candidate in range(index - 1, -1, -1):
        if lines[candidate].strip():
            return lines[candidate].strip()
    return ""


def cpp_function_starts(lines: list[str]) -> list[tuple[int, str]]:
    """Find guarded C/C++ function-definition starts and extracted names."""
    starts = []
    signature: list[str] = []
    signature_start = 0
    in_macro = False
    for index, line in enumerate(lines):
        stripped = line.strip()
        if in_macro:
            in_macro = stripped.endswith("\\")
            signature = []
            continue
        if stripped.startswith("#define") and stripped.endswith("\\"):
            in_macro = True
            signature = []
            continue
        if not stripped or stripped.startswith(("//", "/*", "*", "///", "//!")):
            if signature:
                signature = []
            continue
        if not signature:
            signature_start = index
        signature.append(stripped)
        if len(signature) > 10:
            signature.pop(0)
            signature_start += 1
        if "{" not in stripped:
            if stripped.endswith((";", "}")) or stripped.startswith("#"):
                signature = []
            continue
        joined = " ".join(signature)
        match = CPP_DEFINITION.search(joined)
        if match:
            name = match.group("name").split("::")[-1].replace("operator ", "operator")
            if name not in CPP_CONTROL_WORDS and not joined.startswith(
                ("class ", "struct ", "namespace ", "enum ")
            ) and "](" not in joined:
                starts.append((signature_start, name))
        signature = []
    return starts


def deduplicate_cpp_comments(lines: list[str]) -> list[str]:
    """Collapse repeated annotation blocks left by an interrupted older pass."""
    collapsed = []
    for line in lines:
        if (
            collapsed
            and line == collapsed[-1]
            and line.lstrip().startswith("///")
            and line.lstrip().strip() != "///"
        ):
            continue
        collapsed.append(line)
    lines = collapsed
    for function_index, _ in reversed(cpp_function_starts(lines)):
        while True:
            nearest_end = function_index - 1
            while nearest_end >= 0 and not lines[nearest_end].strip():
                nearest_end -= 1
            if nearest_end < 0 or not lines[nearest_end].lstrip().startswith("///"):
                break
            nearest_start = nearest_end
            while (
                nearest_start > 0
                and lines[nearest_start - 1].lstrip().startswith("///")
            ):
                nearest_start -= 1

            older_end = nearest_start - 1
            while older_end >= 0 and not lines[older_end].strip():
                older_end -= 1
            if older_end < 0 or not lines[older_end].lstrip().startswith("///"):
                break
            older_start = older_end
            while older_start > 0 and lines[older_start - 1].lstrip().startswith("///"):
                older_start -= 1
            nearest = [line.strip() for line in lines[nearest_start:nearest_end + 1]]
            older = [line.strip() for line in lines[older_start:older_end + 1]]
            if nearest != older:
                break
            removed = nearest_start - older_start
            del lines[older_start:nearest_start]
            function_index -= removed
    return lines


def annotate_cpp(path: Path, text: str, result: Result) -> str:
    """Add Doxygen purpose comments before undocumented C/C++ definitions."""
    lines = deduplicate_cpp_comments(text.splitlines(keepends=True))
    for function_index, name in reversed(cpp_function_starts(lines)):
        previous_index = function_index - 1
        while previous_index >= 0 and not lines[previous_index].strip():
            previous_index -= 1
        expected = f"/// {function_description(name)}"
        if previous_index >= 0 and lines[previous_index].strip() == expected:
            del lines[previous_index + 1:function_index]
    insertions = []
    for index, name in cpp_function_starts(lines):
        result.functions_seen += 1
        previous_index = index - 1
        while previous_index >= 0 and not lines[previous_index].strip():
            previous_index -= 1
        previous = lines[previous_index].strip() if previous_index >= 0 else ""
        description = function_description(name)
        if not should_annotate_function(name, description):
            continue
        if previous.startswith("/// Perform the "):
            indent = re.match(r"\s*", lines[previous_index]).group(0)
            replacement = f"{indent}/// {description}\n"
            if lines[previous_index] != replacement:
                lines[previous_index] = replacement
            continue
        if previous.startswith(("//", "/*", "*", "///", "//!")):
            continue
        indent = re.match(r"\s*", lines[index]).group(0)
        wrapped = textwrap.wrap(description, width=max(40, 96 - len(indent) - 4))
        insertions.append((index, [f"{indent}/// {part}\n" for part in wrapped]))
    for index, content in sorted(insertions, reverse=True):
        lines[index:index] = content
    result.functions_added += len(insertions)
    output = "".join(lines)
    if MARKER not in output:
        updated = output.splitlines(keepends=True)
        index = cpp_comment_insertion_line(updated)
        updated[index:index] = [f"// {MARKER} {annotation_for(path)}\n"]
        output = "".join(updated)
    return output


JS_DEFINITION = re.compile(
    r"^(?P<indent>\s*)(?:async\s+)?function\s+(?P<name>[A-Za-z_$][\w$]*)\s*\(|"
    r"^(?P<arrow_indent>\s*)(?:const|let)\s+(?P<arrow>[A-Za-z_$][\w$]*)\s*=.*=>"
)


def annotate_javascript(path: Path, text: str, result: Result) -> str:
    """Add JSDoc purpose comments before named JavaScript functions."""
    lines = text.splitlines(keepends=True)
    insertions = []
    for index, line in enumerate(lines):
        match = JS_DEFINITION.search(line)
        if not match:
            continue
        result.functions_seen += 1
        if previous_code_line(lines, index).startswith(("//", "/*", "*", "/**")):
            continue
        name = match.group("name") or match.group("arrow")
        indent = match.group("indent") or match.group("arrow_indent") or ""
        description = function_description(name)
        if not should_annotate_function(name, description):
            continue
        insertions.append((index, [f"{indent}/** {description} */\n"]))
    for index, content in sorted(insertions, reverse=True):
        lines[index:index] = content
    result.functions_added += len(insertions)
    output = "".join(lines)
    if MARKER not in output:
        output = f"// {MARKER} {annotation_for(path)}\n" + output
    return output


SHELL_FUNCTION = re.compile(
    r"^(?P<indent>\s*)(?:function\s+)?(?P<name>[A-Za-z_]\w*)\s*\(\)\s*\{"
)


def annotate_shell(path: Path, text: str, result: Result) -> str:
    """Add purpose comments before named shell functions."""
    lines = text.splitlines(keepends=True)
    insertions = []
    for index, line in enumerate(lines):
        match = SHELL_FUNCTION.search(line)
        if not match:
            continue
        result.functions_seen += 1
        if previous_code_line(lines, index).startswith("#"):
            continue
        description = function_description(match.group("name"))
        if not should_annotate_function(match.group("name"), description):
            continue
        insertions.append(
            (
                index,
                [
                    f"{match.group('indent')}# {description}\n"
                ],
            )
        )
    for index, content in sorted(insertions, reverse=True):
        lines[index:index] = content
    result.functions_added += len(insertions)
    output = "".join(lines)
    if MARKER not in output:
        updated = output.splitlines(keepends=True)
        index = comment_insertion_line(updated)
        updated[index:index] = [f"# {MARKER} {annotation_for(path)}\n"]
        output = "".join(updated)
    return output


def annotate_config(path: Path, text: str) -> str:
    """Add a format-safe role annotation to XML, YAML, CFG, or CMake files."""
    if MARKER in text:
        return text
    description = annotation_for(path)
    if path.suffix.lower() in {".html", ".xacro", ".xml"}:
        lines = text.splitlines(keepends=True)
        index = 1 if lines and lines[0].lstrip().startswith("<?xml") else 0
        lines[index:index] = [f"<!-- {MARKER} {description} -->\n"]
        return "".join(lines)
    return f"# {MARKER} {description}\n{text}"


CMAKE_BUILD_NOTE = "CARKit build note:"
CMAKE_SECTIONS = (
    (
        re.compile(r"^\s*project\s*\(", re.IGNORECASE),
        "sets the package identity and compiler languages used below.",
    ),
    (
        re.compile(r"^\s*find_package\s*\(", re.IGNORECASE),
        "resolves build-time dependencies and makes their CMake targets available.",
    ),
    (
        re.compile(r"^\s*rosidl_generate_interfaces\s*\(", re.IGNORECASE),
        "generates ROS language bindings for the listed message and service definitions.",
    ),
    (
        re.compile(r"^\s*add_(?:executable|library)\s*\(", re.IGNORECASE),
        "defines a compiled target and the source files that implement it.",
    ),
    (
        re.compile(r"^\s*(?:ament_target_dependencies|target_link_libraries)\s*\(", re.IGNORECASE),
        "attaches ROS dependencies and their transitive include/link settings to the target.",
    ),
    (
        re.compile(r"^\s*install\s*\(", re.IGNORECASE),
        "copies runtime targets and resources into the ROS install space.",
    ),
    (
        re.compile(r"^\s*ament_(?:export_dependencies|export_targets)\s*\(", re.IGNORECASE),
        "exports dependency information for packages that consume this package.",
    ),
    (
        re.compile(r"^\s*ament_package\s*\(", re.IGNORECASE),
        "finalizes the ament package after all targets and install rules are declared.",
    ),
)


def annotate_cmake(path: Path, text: str) -> str:
    """Explain the non-obvious phases of an ament CMake package."""
    lines = text.splitlines(keepends=True)
    seen_patterns: set[int] = set()
    insertions = []
    for index, line in enumerate(lines):
        if line.lstrip().startswith("#"):
            continue
        for pattern_index, (pattern, description) in enumerate(CMAKE_SECTIONS):
            if pattern_index in seen_patterns or not pattern.search(line):
                continue
            seen_patterns.add(pattern_index)
            previous = previous_code_line(lines, index)
            if not previous.startswith("#"):
                insertions.append((index, [f"# {CMAKE_BUILD_NOTE} {description}\n"]))
            break
    for index, content in sorted(insertions, reverse=True):
        lines[index:index] = content
    output = "".join(lines)
    if MARKER not in output:
        output = f"# {MARKER} {annotation_for(path)}\n" + output
    return output


def annotate_css(path: Path, text: str) -> str:
    """Add a stylesheet role comment without changing CSS behavior."""
    if MARKER in text:
        return text
    return f"/* {MARKER} {annotation_for(path)} */\n{text}"


def clean_test_annotations(path: Path, text: str) -> str:
    """Remove only annotations generated by this tool from test-only files."""
    suffix = path.suffix.lower()
    if suffix == ".py":
        tree = ast.parse(text, filename=str(path))
        lines = text.splitlines(keepends=True)
        edits = []
        for node in ast.walk(tree):
            if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) or not node.body:
                continue
            description = ast.get_docstring(node, clean=False)
            if description is None or not looks_generated_description(node.name, description):
                continue
            statement = node.body[0]
            edits.append((statement.lineno - 1, statement.end_lineno))
        for start, end in sorted(edits, reverse=True):
            del lines[start:end]
        return "".join(line for line in lines if MARKER not in line)

    lines = text.splitlines(keepends=True)
    if suffix in {".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp"}:
        for function_index, name in reversed(cpp_function_starts(lines)):
            end = function_index
            start = end
            while start > 0 and lines[start - 1].lstrip().startswith("///"):
                start -= 1
            if start == end:
                continue
            description = " ".join(
                line.lstrip().removeprefix("///").strip() for line in lines[start:end]
            )
            if looks_generated_description(name, description):
                del lines[start:end]
    elif suffix == ".js":
        for index in range(len(lines) - 1, 0, -1):
            match = JS_DEFINITION.search(lines[index])
            if not match:
                continue
            name = match.group("name") or match.group("arrow")
            previous = lines[index - 1].strip()
            if previous.startswith("/** ") and previous.endswith(" */"):
                description = previous[4:-3]
                if looks_generated_description(name, description):
                    del lines[index - 1]
    elif suffix in {".sh", ".bash"}:
        for index in range(len(lines) - 1, 0, -1):
            match = SHELL_FUNCTION.search(lines[index])
            if not match:
                continue
            previous = lines[index - 1].strip().removeprefix("# ")
            if looks_generated_description(match.group("name"), previous):
                del lines[index - 1]
    generated_prefixes = (
        "/// Perform the ",
        "/// Initialize a ",
        "/** Perform the ",
        "/** Initialize a ",
        "# Perform the ",
        "# Initialize a ",
    )
    return "".join(
        line
        for line in lines
        if MARKER not in line and not line.strip().startswith(generated_prefixes)
    )


def clean_tests(root: Path, write: bool) -> int:
    """Remove generated annotations from tests while preserving their source."""
    changed = 0
    for relative_root in ROOTS:
        directory = root / relative_root
        if not directory.exists():
            continue
        for path in sorted(directory.rglob("*")):
            if (
                not path.is_file()
                or any(part in SKIPPED_PARTS for part in path.parts)
                or not is_test_path(path.relative_to(root))
                or not supported_source(path)
            ):
                continue
            original = read_source(path)
            updated = clean_test_annotations(path, original)
            if updated == original:
                continue
            changed += 1
            if write:
                write_source(path, updated)
    return changed


def clean_generic_annotations(root: Path, write: bool) -> int:
    """Remove low-value generated comments that only restate opaque names."""
    changed = 0
    for path in source_files(root):
        original = read_source(path)
        suffix = path.suffix.lower()
        lines = original.splitlines(keepends=True)
        if suffix == ".py":
            tree = ast.parse(original, filename=str(path))
            edits = []
            for node in ast.walk(tree):
                if not isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) or not node.body:
                    continue
                description = ast.get_docstring(node, clean=False)
                if (
                    not description
                    or not looks_generated_description(node.name, description)
                    or should_annotate_function(node.name, description)
                ):
                    continue
                statement = node.body[0]
                edits.append((statement.lineno - 1, statement.end_lineno))
            for start, end in sorted(edits, reverse=True):
                del lines[start:end]
        elif suffix in {".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp"}:
            generated_by_name = {
                description
                for _, name in cpp_function_starts(lines)
                for description in generated_descriptions(name)
                if not should_annotate_function(name, description)
            }
            for function_index, name in reversed(cpp_function_starts(lines)):
                end = function_index
                start = end
                while start > 0 and lines[start - 1].lstrip().startswith("///"):
                    start -= 1
                if start == end:
                    continue
                description = " ".join(
                    line.lstrip().removeprefix("///").strip() for line in lines[start:end]
                )
                if (
                    looks_generated_description(name, description)
                    and not should_annotate_function(name, description)
                ):
                    del lines[start:end]
            lines = [
                line for line in lines
                if not (
                    line.lstrip().startswith("///")
                    and line.lstrip().removeprefix("///").strip() in generated_by_name
                )
            ]
        elif suffix == ".js":
            removals = set()
            for index, line in enumerate(lines):
                match = JS_DEFINITION.search(line)
                if not match or index == 0:
                    continue
                name = match.group("name") or match.group("arrow")
                description = function_description(name)
                if not should_annotate_function(name, description):
                    removals.add(f"/** {description} */")
            lines = [line for line in lines if line.strip() not in removals]
        elif suffix in {".sh", ".bash"}:
            removals = set()
            for line in lines:
                match = SHELL_FUNCTION.search(line)
                if not match:
                    continue
                name = match.group("name")
                description = function_description(name)
                if not should_annotate_function(name, description):
                    removals.add(f"# {description}")
            lines = [line for line in lines if line.strip() not in removals]
        updated = "".join(lines)
        if updated == original:
            continue
        changed += 1
        if write:
            write_source(path, updated)
    return changed


def transform(path: Path, text: str, result: Result) -> str:
    """Dispatch one source file to its format-aware annotator."""
    suffix = path.suffix.lower()
    if path.name == "CMakeLists.txt":
        return annotate_cmake(path, text)
    if suffix == ".py":
        return annotate_python(path, text, result)
    if suffix in {".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp"}:
        return annotate_cpp(path, text, result)
    if suffix == ".js":
        return annotate_javascript(path, text, result)
    if suffix in {".sh", ".bash"}:
        return annotate_shell(path, text, result)
    if suffix == ".css":
        return annotate_css(path, text)
    return annotate_config(path, text)


def run(root: Path, write: bool) -> Result:
    """Annotate or audit every supported file and return aggregate counts."""
    result = Result()
    for path in source_files(root):
        result.files_seen += 1
        original = read_source(path)
        updated = transform(path, original, result)
        if updated == original:
            continue
        result.files_changed += 1
        if write:
            write_source(path, updated)
    return result


def main() -> int:
    """Parse command-line options, run the annotator, and print its audit."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--write", action="store_true", help="apply missing annotations")
    parser.add_argument(
        "--clean-tests",
        action="store_true",
        help="remove this tool's annotations from test-only files",
    )
    parser.add_argument(
        "--clean-generic",
        action="store_true",
        help="remove low-value name-restating annotations",
    )
    parser.add_argument(
        "--root", type=Path, default=Path(__file__).resolve().parents[1],
        help="CARKit repository root",
    )
    args = parser.parse_args()
    if args.clean_tests:
        changed = clean_tests(args.root, args.write)
        mode = "cleaned" if args.write else "would clean"
        print(f"{mode} generated annotations from {changed} test files")
        return 0
    if args.clean_generic:
        changed = clean_generic_annotations(args.root, args.write)
        mode = "cleaned" if args.write else "would clean"
        print(f"{mode} generic annotations from {changed} files")
        return 0
    result = run(args.root, args.write)
    mode = "annotated" if args.write else "would annotate"
    print(
        f"{mode} {result.files_changed}/{result.files_seen} files; "
        f"functions seen={result.functions_seen}, "
        f"function comments added={result.functions_added}, "
        f"unhandled={result.functions_missing}"
    )
    return 1 if result.functions_missing else 0


if __name__ == "__main__":
    raise SystemExit(main())
