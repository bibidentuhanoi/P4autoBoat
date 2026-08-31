import subprocess, tempfile, unittest
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]


class TrimLearnTest(unittest.TestCase):
    def test_learner_rules(self):
        with tempfile.TemporaryDirectory() as tmp:
            binary = Path(tmp) / "trim_learn_test"
            subprocess.run([
                "cc", "-std=c11", "-Wall", "-Wextra", "-Werror",
                "-I", str(ROOT / "main"),
                str(ROOT / "main" / "trim_learn.c"),
                str(ROOT / "tests" / "test_trim_learn.c"),
                "-lm", "-o", str(binary),
            ], check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    unittest.main()
