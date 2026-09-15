#!/usr/bin/env python3
"""Contract tests for the real-robot tracking Word report builder."""

import importlib.util
import sys
import unittest
import zipfile
from pathlib import Path
from tempfile import TemporaryDirectory
from xml.etree import ElementTree

from docx import Document
from pptx import Presentation


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
    def test_conditions_explicitly_exclude_historical_recorder_radius(self):
        builder = load_builder()
        conditions, _ = builder.load_report_data(
            REPOSITORY_ROOT / "docs/reports/2026-09-15-real-robot-tracking")
        for row in conditions:
            self.assertIn("recording.ideal_radius_m=2.0", row["备注"])
            self.assertIn("不参与本次误差计算", row["备注"])
            self.assertIn("控制器 radius 1.0 m", row["备注"])

    def test_stage_results_include_the_same_hpc_representative_as_figures(self):
        builder = load_builder()
        _, metrics = builder.load_report_data(
            REPOSITORY_ROOT / "docs/reports/2026-09-15-real-robot-tracking")
        rows = builder.metric_rows(metrics, "hpc_lpc_stage_result")
        self.assertEqual(len(rows), 3)
        self.assertEqual(rows[0][0], "Artstein-HPC（Leader 命令前馈：开）")
        self.assertEqual(rows[0][2:], ("0.1202", "0.2791", "0.0355"))

    def test_deliverables_expose_radius_procedure_and_traceable_figure_sources(self):
        builder = load_builder()
        input_dir = REPOSITORY_ROOT / "docs/reports/2026-09-15-real-robot-tracking"
        _, metrics = builder.load_report_data(input_dir)
        with TemporaryDirectory() as temporary_directory:
            word_out = Path(temporary_directory) / "report.docx"
            ppt_out = Path(temporary_directory) / "report.pptx"
            builder.build_word(input_dir, word_out)
            builder.build_presentation(input_dir, ppt_out)
            document = Document(word_out)
            word_text = "\n".join(
                [p.text for p in document.paragraphs]
                + [cell.text for table in document.tables for row in table.rows for cell in row.cells])
            slides = Presentation(ppt_out).slides
            slide_texts = ["\n".join(shape.text for shape in slide.shapes if shape.has_text_frame)
                           for slide in slides]
            ppt_text = "\n".join(slide_texts)
            for slide_index, prefix in ((4, "共同配置"), (5, "开启："), (6, "HPC 代表组")):
                body = next(shape for shape in slides[slide_index].shapes
                            if shape.has_text_frame and shape.text.startswith(prefix))
                self.assertTrue(body.text_frame.word_wrap)
            for name, text in (("Word", word_text), ("PPT slide 5", slide_texts[4])):
                with self.subTest(deliverable=name):
                    for required in ("recording.ideal_radius_m=2.0", "不参与本次误差计算",
                                     "radius=1.0 m", "/robot1", "/robot2", "tau=0.43 s", "Td=0.22 s",
                                     "准备", "启动", "录制 45 s", "停止"):
                        self.assertIn(required, text)
            for row in metrics:
                for name, text in (("Word", word_text), ("PPT", ppt_text)):
                    with self.subTest(deliverable=name, experiment=row["experiment_id"]):
                        self.assertIn(row["source_dir"], text)
                        self.assertIn(row["trial_id"], text)
            for reference in ("来源 A、B", "来源 A、C、D"):
                self.assertIn(reference, word_text)
                self.assertIn(reference, ppt_text)
            for banned in builder.BANNED_TERMS:
                self.assertNotIn(banned, word_text)

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

    def test_presentation_has_nine_editable_slides_and_a_shape_only_video_placeholder(self):
        builder = load_builder()
        input_dir = REPOSITORY_ROOT / "docs" / "reports" / "2026-09-15-real-robot-tracking"

        with TemporaryDirectory() as temporary_directory:
            ppt_out = Path(temporary_directory) / "report.pptx"
            builder.build_presentation(input_dir, ppt_out)

            with zipfile.ZipFile(ppt_out) as archive:
                slide_names = sorted(
                    name for name in archive.namelist()
                    if name.startswith("ppt/slides/slide") and name.endswith(".xml")
                )
                self.assertEqual(len(slide_names), 9)
                video_extensions = (".mp4", ".mov", ".avi", ".wmv", ".webm", ".m4v")
                self.assertFalse(any("video" in name.lower() or name.lower().endswith(video_extensions)
                                     for name in archive.namelist()))
                slide_eight_relationships = archive.read("ppt/slides/_rels/slide8.xml.rels").decode()
                self.assertNotIn("video", slide_eight_relationships.lower())
                self.assertNotIn("TargetMode=\"External\"", slide_eight_relationships)
                slide_eight = ElementTree.fromstring(archive.read("ppt/slides/slide8.xml"))

            namespaces = {"a": "http://schemas.openxmlformats.org/drawingml/2006/main"}
            slide_eight_text = "".join(node.text or "" for node in slide_eight.findall(".//a:t", namespaces))
            self.assertIn("请在此处手动插入最新实物实验视频", slide_eight_text)
            self.assertIn("Artstein-LPC，λ初值=2.5，Leader 速度=0.25 m/s", slide_eight_text)
            self.assertGreaterEqual(len(slide_eight.findall(".//p:sp", {
                "p": "http://schemas.openxmlformats.org/presentationml/2006/main"
            })), 3)
            slide_eight_shapes = Presentation(ppt_out).slides[7].shapes
            self.assertTrue(any(
                abs((shape.width / shape.height) - (16 / 9)) < 0.001
                for shape in slide_eight_shapes if shape.height
            ))


if __name__ == "__main__":
    unittest.main()
