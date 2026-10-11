from pathlib import Path
import re

from docx import Document
from docx.enum.table import WD_TABLE_ALIGNMENT, WD_CELL_VERTICAL_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Cm, Pt, RGBColor


ROOT = Path(__file__).resolve().parent
TEMPLATE = Path('/home/junecyo/.codex/attachments/61e0614d-8c71-4fb4-847f-f0833b721280/2026智慧社区复赛技术报告模板_1789957534837.docx')
SOURCE = ROOT / 'report_body.md'
OUTPUT = ROOT / '智慧社区复赛技术报告.docx'

ABSTRACT = ('本项目面向智慧社区巡检任务，构建两轮小车的闭环仿真系统。小车从起点出发，按十六个航点通过两组信号灯，'
            '在两块人偶立牌前依次停车计数，于第二块立牌完成后汇总人数；随后在三辆车前分别检测车牌、读取文字并保存结果，最后返回起点。'
            '系统结合地图、激光扫描、里程计和相机完成导航与识别，对未确认结果单独标记，并在暗光与强光下按亮度调整图像。'
            '一次完整仿真运行用时一百七十点九秒，两块立牌分别记录四人和五人，三处车牌文字均获确认。报告给出模块顺序、测试记录、已知问题及实车迁移步骤。')


def set_font(run, name='Noto Serif CJK SC', size=10.5, bold=False, color=None):
    run.font.name = name
    run.font.size = Pt(size)
    run.font.bold = bold
    if color:
        run.font.color.rgb = RGBColor(*color)
    rpr = run._element.get_or_add_rPr()
    fonts = rpr.rFonts
    if fonts is None:
        fonts = OxmlElement('w:rFonts')
        rpr.insert(0, fonts)
    fonts.set(qn('w:eastAsia'), name)


def add_text(paragraph, value, **kwargs):
    set_font(paragraph.add_run(value), **kwargs)


def body_paragraph(doc, value):
    p = doc.add_paragraph()
    p.paragraph_format.first_line_indent = Cm(0.74)
    p.paragraph_format.line_spacing = 1.35
    p.paragraph_format.space_after = Pt(6)
    p.paragraph_format.widow_control = True
    add_text(p, value)
    return p


def heading(doc, value, level):
    p = doc.add_paragraph()
    p.style = 'Normal'
    p.paragraph_format.keep_with_next = True
    p.paragraph_format.space_before = Pt(15 if level == 1 else 10)
    p.paragraph_format.space_after = Pt(8 if level == 1 else 5)
    p.paragraph_format.line_spacing = 1.2
    add_text(p, value, name='Noto Sans CJK SC', size=15 if level == 1 else (12.5 if level == 2 else 11),
             bold=True, color=(25, 58, 79))


def shade(cell, fill):
    tcpr = cell._tc.get_or_add_tcPr()
    shd = OxmlElement('w:shd')
    shd.set(qn('w:fill'), fill)
    tcpr.append(shd)


def add_table(doc, rows):
    table = doc.add_table(rows=0, cols=len(rows[0]))
    table.alignment = WD_TABLE_ALIGNMENT.CENTER
    table.autofit = True
    table.style = 'Table Grid'
    for r, values in enumerate(rows):
        cells = table.add_row().cells
        for c, value in enumerate(values):
            cell = cells[c]
            cell.vertical_alignment = WD_CELL_VERTICAL_ALIGNMENT.CENTER
            p = cell.paragraphs[0]
            p.paragraph_format.space_after = Pt(1)
            p.paragraph_format.space_before = Pt(1)
            p.paragraph_format.line_spacing = 1.1
            add_text(p, value, name='Noto Sans CJK SC', size=8.7, bold=r == 0)
            if r == 0:
                shade(cell, 'DCEBF2')
    doc.add_paragraph().paragraph_format.space_after = Pt(2)


def add_figure(doc, alt, relative):
    image = ROOT / relative
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    p.paragraph_format.space_before = Pt(7)
    p.paragraph_format.space_after = Pt(4)
    p.add_run().add_picture(str(image), width=Cm(15.4))
    caption = doc.add_paragraph()
    caption.alignment = WD_ALIGN_PARAGRAPH.CENTER
    caption.paragraph_format.space_after = Pt(9)
    add_text(caption, alt, name='Noto Sans CJK SC', size=9)


def build():
    doc = Document(TEMPLATE)
    cover = doc.tables[0]
    for row in range(3):
        cover.cell(row, 1).text = ''
    cover.cell(3, 1).text = ABSTRACT
    for cell in cover.cell(3, 1).paragraphs:
        cell.paragraph_format.line_spacing = 1.25
        cell.paragraph_format.space_after = Pt(1)
        for run in cell.runs:
            set_font(run, size=10)
    body = doc._element.body
    for child in list(body)[6:]:
        if child.tag != qn('w:sectPr'):
            body.remove(child)
    doc.add_page_break()

    lines = SOURCE.read_text(encoding='utf-8').splitlines()
    i = 0
    while i < len(lines):
        line = lines[i].strip()
        i += 1
        if not line:
            continue
        if line == '<!-- pagebreak -->':
            doc.add_page_break()
            continue
        if line.startswith('```'):
            block = []
            while i < len(lines) and not lines[i].startswith('```'):
                block.append(lines[i])
                i += 1
            i += 1
            for value in block:
                p = doc.add_paragraph()
                p.paragraph_format.left_indent = Cm(0.4)
                p.paragraph_format.space_after = Pt(2)
                add_text(p, value, name='Noto Sans CJK SC', size=8.8)
            continue
        image = re.fullmatch(r'!\[(.+?)\]\((.+?)\)', line)
        if image:
            add_figure(doc, image.group(1), image.group(2))
            continue
        if line.startswith('|'):
            rows = []
            while line.startswith('|'):
                fields = [v.strip() for v in line.strip('|').split('|')]
                if not all(re.fullmatch(r'[-: ]+', v or '-') for v in fields):
                    rows.append(fields)
                if i >= len(lines):
                    break
                line = lines[i].strip()
                i += 1
            if line and not line.startswith('|'):
                i -= 1
            add_table(doc, rows)
            continue
        if line.startswith('#'):
            level = len(line) - len(line.lstrip('#'))
            heading(doc, line[level:].strip(), min(level, 3))
            continue
        body_paragraph(doc, line)

    doc.core_properties.title = '智慧社区复赛技术报告'
    doc.core_properties.subject = '智慧社区机器人巡检仿真与视觉识别'
    doc.core_properties.keywords = '智慧社区,机器人,仿真,车牌识别,人偶计数'
    doc.save(OUTPUT)
    print(OUTPUT)


if __name__ == '__main__':
    build()
