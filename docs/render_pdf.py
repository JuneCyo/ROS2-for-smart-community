#!/usr/bin/env python3
"""Render the editable HTML technical plan as a paginated PDF."""

from html import escape
from pathlib import Path

from bs4 import BeautifulSoup, NavigableString, Tag
from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER, TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.cidfonts import UnicodeCIDFont
from reportlab.platypus import (
    CondPageBreak, HRFlowable, Image, KeepTogether, PageBreak, Paragraph, SimpleDocTemplate,
    Spacer, Table, TableStyle,
)


HERE = Path(__file__).resolve().parent
SOURCE = HERE / "智慧社区复赛技术方案.html"
OUTPUT = HERE / "智慧社区复赛技术方案.pdf"

pdfmetrics.registerFont(UnicodeCIDFont("STSong-Light"))
FONT = "STSong-Light"
styles = {
    "title": ParagraphStyle("title", fontName=FONT, fontSize=22, leading=31,
                            alignment=TA_CENTER, textColor=colors.HexColor("#183d58"), spaceAfter=8 * mm),
    "subtitle": ParagraphStyle("subtitle", fontName=FONT, fontSize=13, leading=20,
                               alignment=TA_CENTER, textColor=colors.HexColor("#536b7b"), spaceAfter=4 * mm),
    "meta": ParagraphStyle("meta", fontName=FONT, fontSize=8.8, leading=14,
                           alignment=TA_CENTER, textColor=colors.HexColor("#536b7b"), spaceAfter=7 * mm),
    "h2": ParagraphStyle("h2", fontName=FONT, fontSize=13.5, leading=19,
                         textColor=colors.HexColor("#183d58"), spaceBefore=4 * mm, spaceAfter=1 * mm,
                         keepWithNext=True),
    "h3": ParagraphStyle("h3", fontName=FONT, fontSize=10.7, leading=15,
                         textColor=colors.HexColor("#183d58"), spaceBefore=2.5 * mm, spaceAfter=1 * mm,
                         keepWithNext=True),
    "body": ParagraphStyle("body", fontName=FONT, fontSize=9.1, leading=14,
                           alignment=TA_LEFT, spaceAfter=1.6 * mm, wordWrap="CJK"),
    "note": ParagraphStyle("note", fontName=FONT, fontSize=9.2, leading=15,
                           backColor=colors.HexColor("#f2f6f8"), borderColor=colors.HexColor("#46768f"),
                           borderWidth=0.6, borderPadding=7, spaceAfter=3 * mm, wordWrap="CJK"),
    "small": ParagraphStyle("small", fontName=FONT, fontSize=8, leading=13,
                            textColor=colors.HexColor("#536b7b"), spaceAfter=2 * mm, wordWrap="CJK"),
    "caption": ParagraphStyle("caption", fontName=FONT, fontSize=8.2, leading=12,
                              alignment=TA_CENTER, textColor=colors.HexColor("#526778"), spaceAfter=4 * mm),
    "cell": ParagraphStyle("cell", fontName=FONT, fontSize=8, leading=11.5, wordWrap="CJK"),
    "code": ParagraphStyle("code", fontName=FONT, fontSize=8.1, leading=13.5,
                           backColor=colors.HexColor("#f2f5f7"), borderPadding=7, spaceAfter=3 * mm),
}


def inline(node):
    if isinstance(node, NavigableString):
        return escape(str(node))
    if not isinstance(node, Tag):
        return ""
    content = "".join(inline(child) for child in node.children)
    if node.name in ("b", "strong"):
        return f"<b>{content}</b>"
    if node.name == "code":
        return f'<font color="#295978">{content}</font>'
    if node.name == "br":
        return "<br/>"
    return content


def para(node, style="body"):
    return Paragraph("".join(inline(child) for child in node.children), styles[style])


def figure(node):
    img = node.find("img")
    path = (HERE / img["src"]).resolve()
    item = Image(str(path))
    max_width = 175 * mm
    max_height = 80 * mm if img.get("class") == ["map"] else 125 * mm if img.get("class") == ["scene"] else 120 * mm
    scale = min(max_width / item.drawWidth, max_height / item.drawHeight, 1)
    item.drawWidth *= scale
    item.drawHeight *= scale
    item.hAlign = "CENTER"
    caption = node.find("figcaption")
    return KeepTogether([item, Spacer(1, 1.5 * mm), para(caption, "caption")])


def table(node):
    rows = []
    for tr in node.find_all("tr", recursive=False):
        rows.append([para(cell, "cell") for cell in tr.find_all(["th", "td"], recursive=False)])
    count = len(rows[0])
    widths = ([80, 130, 273] if count == 3 else [115, 368])
    result = Table(rows, colWidths=widths, repeatRows=1, hAlign="LEFT")
    result.setStyle(TableStyle([
        ("GRID", (0, 0), (-1, -1), 0.45, colors.HexColor("#aab9c5")),
        ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#e9f1f6")),
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
        ("LEFTPADDING", (0, 0), (-1, -1), 6),
        ("RIGHTPADDING", (0, 0), (-1, -1), 6),
        ("TOPPADDING", (0, 0), (-1, -1), 3),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 3),
    ]))
    return result


def footer(canvas, doc):
    canvas.saveState()
    canvas.setFont(FONT, 8)
    canvas.setFillColor(colors.HexColor("#70818c"))
    canvas.drawString(17 * mm, 11 * mm, "智慧社区复赛技术方案")
    canvas.drawRightString(A4[0] - 17 * mm, 11 * mm, str(doc.page))
    canvas.restoreState()


def main():
    soup = BeautifulSoup(SOURCE.read_text(encoding="utf-8"), "html.parser")
    story = []
    for node in soup.body.children:
        if not isinstance(node, Tag):
            continue
        if node.name == "div" and "cover" in node.get("class", []):
            for child in node.children:
                if not isinstance(child, Tag):
                    continue
                if child.name == "h1":
                    story.append(para(child, "title"))
                elif child.name == "p":
                    style = "subtitle" if "subtitle" in child.get("class", []) else "meta"
                    story.append(para(child, style))
                elif child.name == "figure":
                    story.append(figure(child))
            story.append(PageBreak())
        elif node.name == "h2":
            story.append(CondPageBreak(65))
            story.append(KeepTogether([
                para(node, "h2"),
                HRFlowable(width="100%", thickness=0.6,
                           color=colors.HexColor("#9db1bf")),
                Spacer(1, 4 * mm),
            ]))
        elif node.name == "h3":
            story.append(para(node, "h3"))
        elif node.name == "p":
            classes = node.get("class", [])
            story.append(para(node, "note" if "note" in classes else "small" if "small" in classes else "body"))
        elif node.name == "table":
            story.extend([table(node), Spacer(1, 3 * mm)])
        elif node.name == "figure":
            story.append(figure(node))
        elif node.name in ("ol", "ul"):
            for number, li in enumerate(node.find_all("li", recursive=False), 1):
                prefix = f"{number}. " if node.name == "ol" else "• "
                story.append(Paragraph(prefix + "".join(inline(child) for child in li.children), styles["body"]))
        elif node.name == "pre":
            lines = [escape(line) for line in node.get_text().strip().splitlines()]
            story.append(Paragraph("<br/>".join(lines), styles["code"]))

    doc = SimpleDocTemplate(str(OUTPUT), pagesize=A4, leftMargin=17 * mm,
                            rightMargin=17 * mm, topMargin=13 * mm, bottomMargin=13 * mm,
                            title="智慧社区复赛技术方案", author="项目技术文档")
    doc.build(story, onFirstPage=footer, onLaterPages=footer)
    print(OUTPUT)


if __name__ == "__main__":
    main()
