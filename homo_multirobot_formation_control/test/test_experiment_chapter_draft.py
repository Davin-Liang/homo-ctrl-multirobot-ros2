import unittest
from pathlib import Path


CHAPTER = Path(__file__).parents[1] / 'doc' / 'experiment_chapter_draft.md'


class ExperimentChapterDraftTest(unittest.TestCase):
    def test_chapter_covers_only_approved_experiments(self):
        source = CHAPTER.read_text(encoding='utf-8')
        for required in (
            '不同齐次度下的编队性能', '齐次控制与线性控制对比',
            '延迟补偿方案消融实验', '距离误差', '相对航向误差',
            '速度差', 'homogeneity_metrics_table.xlsx',
            'homogeneous_vs_linear_xy.png', 'four_compensation_xy.png',
            '不宜据此宣称严格有限时间收敛',
        ):
            self.assertIn(required, source)


if __name__ == '__main__':
    unittest.main()
