#!/usr/bin/env python3
"""Contract tests for the real-robot tracking Word report builder."""

import importlib.util
import sys
import unittest
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
BUILDER_PATH = (
    REPOSITORY_ROOT
    / "homo_multirobot_formation_control"
    / "scripts"
    / "build_real_tracking_deliverables.py"
)


def load_builder():
    spec = importlib.util.spec_from_file_location("report_builder", BUILDER_PATH)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class ReportContentTest(unittest.TestCase):
    def test_content_contract_keeps_required_boundaries_and_excludes_banned_terms(self):
        builder = load_builder()

        paragraphs = builder.report_paragraphs()
        full_text = "\n".join(paragraphs)

        self.assertIn("评价半径固定为 1.0 m", full_text)
        self.assertIn(builder.LPC_SAFETY_BOUNDARY, full_text)
        self.assertIn("Follower 的前向预测", full_text)
        self.assertIn("Leader 命令速度前馈", full_text)
        for banned in builder.BANNED_TERMS:
            self.assertNotIn(banned, full_text)

    def test_input_tables_use_only_the_one_meter_metric_basis(self):
        builder = load_builder()
        input_dir = REPOSITORY_ROOT / "docs" / "reports" / "2026-09-15-real-robot-tracking"

        conditions, metrics = builder.load_report_data(input_dir)

        self.assertEqual(len(conditions), 4)
        self.assertEqual(len(metrics), 4)
        self.assertEqual({row["ideal_radius_m"] for row in metrics}, {"1.0"})


if __name__ == "__main__":
    unittest.main()
