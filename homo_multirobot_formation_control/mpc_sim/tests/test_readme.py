from pathlib import Path


def test_readme_documents_reproducible_full_experiment_command():
    text = (Path(__file__).parents[1] / "README.md").read_text()
    assert "run_experiments.py" in text
    assert "0.30" in text
    assert "6" in text
    assert "CVXPY" in text
