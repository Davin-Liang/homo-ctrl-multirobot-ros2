#!/usr/bin/env python3
"""Build the Chinese Word summary for the 2026-09-15 real-robot study."""

import argparse
import csv
from pathlib import Path

from docx import Document
from docx.enum.section import WD_ORIENT
from docx.enum.table import WD_CELL_VERTICAL_ALIGNMENT, WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Cm, Pt


LPC_SAFETY_BOUNDARY = (
    "LPC 在 initial_min_lambda=1.5、Leader 速度 0.25 m/s 的实物工况下发生碰撞，"
    "未形成可用于轨迹统计的有效记录。LPC 的 λ初值=2.0 记录同时将 Leader 速度降至"
    " 0.20 m/s；λ初值=2.5 记录使用 0.25 m/s。因此现有 HPC/LPC 结果用于说明阶段性"
    "可运行性和现象，不用于宣称严格单变量条件下的性能优劣。"
)
BANNED_TERMS = ("视频", "二维码", "附件", "链接")
ASSET_NAMES = (
    "control_pipeline.png",
    "feedforward_comparison.png",
    "feedforward_distance_error.png",
    "hpc_lpc_trajectory.png",
    "hpc_lpc_distance_error.png",
)


def report_paragraphs():
    """Return body text so factual boundaries can be tested without DOCX parsing."""
    return [
        "双移动机器人 Leader–Follower 实物跟踪实验总结",
        "1. 实验任务说明",
        "本实验在双移动机器人 Leader–Follower 场景中，记录并整理 Artstein-HPC 与 "
        "Artstein-LPC 的阶段性实物跟踪结果。评价半径固定为 1.0 m，所有统计均以该理想"
        "编队半径为唯一评价基准；距离误差定义为实际两车间距减去 1.0 m，表中的末帧误差"
        "为其绝对值。",
        "2. 实物实验平台、数据流与控制方法",
        "两台 mini_omni 全向移动机器人采用动捕状态源，Leader 的轨迹由速度命令驱动，"
        "Follower 依据相对编队误差输出速度命令。控制频率为 20 Hz，记录时长约 45 s。",
        "Follower 的前向预测使用 Follower 当前动捕状态、最近输出命令和电机时间常数 tau "
        "预测其短时状态，以补偿执行器响应；它与 Leader 命令速度前馈是两条不同支路。"
        "Leader 命令速度前馈为可选的 Leader cmd_vel 速度增量支路，直接叠加到控制输出，"
        "不是 Follower 前向预测的开关。",
        "3. 实验过程与统一条件",
        "四组有效记录均来自实物平台和动捕状态源。统计只使用条件表与指标表中理想半径为 "
        "1.0 m 的数据，不根据原始目录名或元数据中的控制器字符串重新推断前馈状态。",
        "4. 实验一：Artstein-HPC 的 Leader 命令速度前馈开/关对比",
        "在两组约 45 s、Leader 平均速度约 0.246 m/s 的 Artstein-HPC 实测记录中，"
        "开启 Leader 命令速度前馈的平均绝对距离误差为 0.1202 m、RMS 距离误差为 "
        "0.2791 m；关闭时分别为 0.1282 m 和 0.2895 m。开启组在这两个汇总指标上较低。"
        "末帧绝对距离误差则为 0.0355 m，高于关闭组的 0.0165 m。因此该组对比仅描述"
        "已记录的实测差异，不将其概括为所有指标的一致改善。",
        "5. 实验二：Artstein-HPC 与 Artstein-LPC 的阶段性结果",
        "LPC 两条有效记录的平均绝对距离误差分别为 0.4001 m 与 0.4144 m，RMS 距离"
        "误差分别为 0.4547 m 与 0.4771 m。对应的 initial_min_lambda 与 Leader 速度"
        "同时变化，结果只作为阶段性观察。",
        LPC_SAFETY_BOUNDARY,
        "6. 阶段性成果、当前问题与下一步工作",
        "已形成统一的实物实验条件表、1.0 m 基准指标表和可复核的图表总结。下一步应在"
        "固定 Leader 速度、初始 λ 和其他运行条件后，补充可重复试验；同时继续核对前馈"
        "支路标注与原始实验配置的一致性。",
    ]


def load_report_data(input_dir):
    """Load the committed report inputs and reject a non-1.0 m metric basis."""
    input_dir = Path(input_dir)
    with (input_dir / "experiment-conditions.csv").open(encoding="utf-8-sig", newline="") as stream:
        conditions = list(csv.DictReader(stream))
    with (input_dir / "metrics.csv").open(encoding="utf-8-sig", newline="") as stream:
        metrics = list(csv.DictReader(stream))
    if not conditions or not metrics:
        raise ValueError("条件表和指标表均不能为空")
    if {row["ideal_radius_m"] for row in metrics} != {"1.0"}:
        raise ValueError("报告只接受理想编队半径为 1.0 m 的指标")
    for asset_name in ASSET_NAMES:
        if not (input_dir / "assets" / asset_name).is_file():
            raise FileNotFoundError(input_dir / "assets" / asset_name)
    return conditions, metrics


def set_cell_shading(cell, fill):
    properties = cell._tc.get_or_add_tcPr()
    shading = OxmlElement("w:shd")
    shading.set(qn("w:fill"), fill)
    properties.append(shading)


def set_run_font(run, size=10.0, bold=False):
    run.font.name = "宋体"
    run._element.rPr.rFonts.set(qn("w:eastAsia"), "宋体")
    run.font.size = Pt(size)
    run.bold = bold


def add_body_paragraph(document, text, *, bold=False):
    paragraph = document.add_paragraph()
    paragraph.paragraph_format.space_after = Pt(5)
    paragraph.paragraph_format.line_spacing = 1.25
    run = paragraph.add_run(text)
    set_run_font(run, bold=bold)
    return paragraph


def add_heading(document, text, level):
    heading = document.add_heading()
    heading.style = document.styles[f"Heading {level}"]
    run = heading.add_run(text)
    set_run_font(run, size=15 if level == 1 else 12, bold=True)
    return heading


def add_caption(document, text):
    paragraph = document.add_paragraph()
    paragraph.alignment = WD_ALIGN_PARAGRAPH.CENTER
    run = paragraph.add_run(text)
    set_run_font(run, size=9)
    return paragraph


def add_image(document, image_path, caption):
    paragraph = document.add_paragraph()
    paragraph.alignment = WD_ALIGN_PARAGRAPH.CENTER
    paragraph.add_run().add_picture(str(image_path), width=Cm(22.5))
    add_caption(document, caption)


def add_table(document, headers, rows):
    table = document.add_table(rows=1, cols=len(headers))
    table.alignment = WD_TABLE_ALIGNMENT.CENTER
    table.style = "Table Grid"
    for cell, header in zip(table.rows[0].cells, headers):
        set_cell_shading(cell, "D9EAF7")
        cell.vertical_alignment = WD_CELL_VERTICAL_ALIGNMENT.CENTER
        paragraph = cell.paragraphs[0]
        paragraph.alignment = WD_ALIGN_PARAGRAPH.CENTER
        run = paragraph.add_run(header)
        set_run_font(run, size=8, bold=True)
    for row in rows:
        cells = table.add_row().cells
        for cell, value in zip(cells, row):
            cell.vertical_alignment = WD_CELL_VERTICAL_ALIGNMENT.CENTER
            paragraph = cell.paragraphs[0]
            paragraph.alignment = WD_ALIGN_PARAGRAPH.CENTER
            run = paragraph.add_run(str(value))
            set_run_font(run, size=7.5)
    document.add_paragraph()
    return table


def metric_rows(metrics, comparison):
    return [
        (
            row["display_label"],
            row["sample_count"],
            f"{float(row['mean_abs_distance_error_m']):.4f}",
            f"{float(row['rms_distance_error_m']):.4f}",
            f"{float(row['final_distance_error_m']):.4f}",
        )
        for row in metrics
        if row["comparison"] == comparison
    ]


def build_word(input_dir, word_out):
    """Build the report document and return its output path."""
    input_dir = Path(input_dir)
    word_out = Path(word_out)
    conditions, metrics = load_report_data(input_dir)
    document = Document()
    section = document.sections[0]
    section.orientation = WD_ORIENT.LANDSCAPE
    section.page_width, section.page_height = section.page_height, section.page_width
    section.top_margin = Cm(1.6)
    section.bottom_margin = Cm(1.6)
    section.left_margin = Cm(1.8)
    section.right_margin = Cm(1.8)
    normal = document.styles["Normal"]
    normal.font.name = "宋体"
    normal._element.rPr.rFonts.set(qn("w:eastAsia"), "宋体")
    normal.font.size = Pt(10)

    title = document.add_paragraph()
    title.alignment = WD_ALIGN_PARAGRAPH.CENTER
    title.paragraph_format.space_after = Pt(14)
    title_run = title.add_run(report_paragraphs()[0])
    set_run_font(title_run, size=18, bold=True)

    paragraphs = report_paragraphs()
    add_heading(document, paragraphs[1], 1)
    add_body_paragraph(document, paragraphs[2])

    add_heading(document, paragraphs[3], 1)
    add_body_paragraph(document, paragraphs[4])
    add_body_paragraph(document, paragraphs[5])
    add_image(document, input_dir / "assets" / "control_pipeline.png", "图 1  实物跟踪控制与数据流")

    add_heading(document, paragraphs[6], 1)
    add_body_paragraph(document, paragraphs[7])
    condition_rows = [
        (
            row["实验标签"], row["控制器"], row["Leader 命令前馈"] or "—",
            row["initial_min_lambda"] or "—", row["Leader 速度"] or "—",
            row["录制时长"], row["状态源"], "1.0 m",
        )
        for row in conditions
    ]
    add_table(document,
              ("实验标签", "控制器", "Leader 命令前馈", "λ初值", "Leader 速度", "时长", "状态源", "评价半径"),
              condition_rows)

    add_heading(document, paragraphs[8], 1)
    add_body_paragraph(document, paragraphs[9])
    add_image(document, input_dir / "assets" / "feedforward_comparison.png", "图 2  Artstein-HPC 前馈开/关轨迹对比")
    add_image(document, input_dir / "assets" / "feedforward_distance_error.png", "图 3  Artstein-HPC 前馈开/关距离误差")
    add_table(document,
              ("实验标签", "采样数", "平均绝对误差 (m)", "RMS 误差 (m)", "末帧绝对误差 (m)"),
              metric_rows(metrics, "leader_command_feedforward"))

    add_heading(document, paragraphs[10], 1)
    add_body_paragraph(document, paragraphs[11])
    add_image(document, input_dir / "assets" / "hpc_lpc_trajectory.png", "图 4  Artstein-HPC 与 Artstein-LPC 阶段性轨迹")
    add_image(document, input_dir / "assets" / "hpc_lpc_distance_error.png", "图 5  Artstein-HPC 与 Artstein-LPC 阶段性距离误差")
    add_table(document,
              ("实验标签", "采样数", "平均绝对误差 (m)", "RMS 误差 (m)", "末帧绝对误差 (m)"),
              metric_rows(metrics, "hpc_lpc_stage_result"))
    add_body_paragraph(document, paragraphs[12])

    add_heading(document, paragraphs[13], 1)
    add_body_paragraph(document, paragraphs[14])
    word_out.parent.mkdir(parents=True, exist_ok=True)
    document.save(word_out)
    return word_out


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path)
    parser.add_argument("--word-out", required=True, type=Path)
    parser.add_argument("--skip-ppt", action="store_true", help="兼容交付命令；本脚本不生成 PPT。")
    return parser.parse_args()


def main():
    args = parse_args()
    output = build_word(args.input, args.word_out)
    print(f"已生成 Word：{output}")


if __name__ == "__main__":
    main()
