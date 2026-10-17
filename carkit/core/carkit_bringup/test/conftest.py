"""Run the same sensor contracts against Python references or native binaries."""
import os
from pathlib import Path
import subprocess


class RuntimeNodes:
    def __init__(self):
        self.nodes = []
        self.processes = []

    def create(self, executable, python_class, executor, arguments=()):
        directory = os.environ.get('CARKIT_NATIVE_RUNTIME_DIR')
        if directory:
            self.processes.append(subprocess.Popen([str(Path(directory)/executable), *arguments],
                                                   stdout=subprocess.DEVNULL))
        else:
            node = python_class()
            executor.add_node(node)
            self.nodes.append(node)

    def close(self):
        for process in self.processes:
            process.terminate()
        for process in self.processes:
            process.wait(timeout=10)
        for node in self.nodes:
            node.destroy_node()
        self.processes.clear()
        self.nodes.clear()


import pytest

@pytest.fixture
def runtime_nodes():
    runtime = RuntimeNodes()
    try:
        yield runtime
    finally:
        runtime.close()
