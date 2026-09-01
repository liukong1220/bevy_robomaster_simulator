#!/usr/bin/env python3
"""可复现地重建 assets/fonts/hud_cjk.otf（HUD 中文子集字体）。

字表必须由 src/ 扫描得到，不能手写：手列字表漏字时 HUD 上是缺字框（□），
Rust 编译期不会有任何提示，只能靠肉眼在运行时发现。

用法：
    python3 scripts/build_hud_font.py             # 重建子集
    python3 scripts/build_hud_font.py --verify    # 只校验现有子集（CI 用）

依赖 fontTools（本机 4.29.1）。源字体默认取本机 fonts-noto-cjk 包里的
NotoSansCJK-Regular.ttc 子字体 2（= NotoSansCJKsc-Regular），可用 --source /
--font-number 覆盖。详见 assets/fonts/README.md。
"""

import argparse
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
SRC_DIR = REPO / "src"
OUT = REPO / "assets" / "fonts" / "hud_cjk.otf"

# ttc 里 10 个子字体共用码点但字形不同，0 号是 JP。不显式指定就会静默取到日文字形：
# 字数不少、也不报错，只是"会""来""令"长成日文写法。
DEFAULT_SOURCE = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
DEFAULT_FONT_NUMBER = 2  # NotoSansCJKsc-Regular
EXPECT_PS_NAME = "NotoSansCJKsc-Regular"

# 扫描的是**字符串与字符字面量**，不是整份源码：注释里的字不会被渲染，
# 把它们一起塞进子集只会白白增大文件。
#
# 这一段必须自己走一遍词法，不能拿正则去扫原始文本。之前的版本就是这么做的：
# 于是 `//` 中文注释里只要出现一个引号，从它到下一个引号之间的注释正文就被当成
# 字符串字面量收进子集。后果有两个方向——改一行注释就让 .otf 变化（无谓的 diff
# 与"字体过期"告警），而真正决定 HUD 渲染的字面量集合反而看不清楚。
RAW_PREFIX = re.compile(r'(?:br|r)(?P<hashes>\#*)"')
CHAR_LITERAL = re.compile(r"'(\\.|[^'\\\n])'", re.DOTALL)
IDENT_CHARS = re.compile(r"[A-Za-z0-9_]")


def _at_token_start(text: str, i: int) -> bool:
    """i 处是不是一个新 token 的开头（用来区分 `r"..."` 与标识符里的 r）。"""
    return i == 0 or not IDENT_CHARS.match(text[i - 1])


def iter_literals(text: str):
    """按 Rust 词法产出字符串/字符字面量的**内容**，跳过注释。

    `b"..."` 不需要单独处理：把前缀 `b` 当普通字符走过去，随后那个 `"` 就按普通
    字符串取到同一段内容。原始字符串必须单独处理，因为它没有转义、且靠 `#` 的
    个数配对结束（`r#"含 " 的文本"#`）。
    """
    i, n = 0, len(text)
    while i < n:
        c = text[i]

        if c == "/" and text.startswith("//", i):
            j = text.find("\n", i)
            i = n if j < 0 else j + 1
            continue

        if c == "/" and text.startswith("/*", i):
            # Rust 的块注释可以嵌套，不能直接 find("*/")。
            depth, i = 1, i + 2
            while i < n and depth:
                if text.startswith("/*", i):
                    depth, i = depth + 1, i + 2
                elif text.startswith("*/", i):
                    depth, i = depth - 1, i + 2
                else:
                    i += 1
            continue

        if c in ("r", "b") and _at_token_start(text, i):
            m = RAW_PREFIX.match(text, i)
            if m:
                close = '"' + m.group("hashes")
                j = text.find(close, m.end())
                if j < 0:  # 未闭合的原始字符串：源码本身不合法，停在这里
                    return
                yield text[m.end() : j]
                i = j + len(close)
                continue

        if c == '"':
            j = i + 1
            while j < n:
                if text[j] == "\\":
                    j += 2
                    continue
                if text[j] == '"':
                    break
                j += 1
            yield text[i + 1 : j]
            i = j + 1
            continue

        if c == "'":
            # 生命周期（`&'a str`）匹配不到闭合引号，会自然落到下面的 i += 1。
            m = CHAR_LITERAL.match(text, i)
            if m:
                yield m.group(1)
                i = m.end()
                continue

        i += 1


def collect_codepoints() -> set[int]:
    """src/ 下所有字符串/字符字面量里的非 ASCII 字符 + 全部可打印 ASCII。"""
    # 可打印 ASCII 无条件全收：HUD 会格式化数字、单位、方向键提示等等，
    # 按"源码里出现过"筛 ASCII 会因为一次 format! 拼接就漏字。
    cps = {c for c in range(0x20, 0x7F)}
    for path in sorted(SRC_DIR.rglob("*.rs")):
        for literal in iter_literals(path.read_text(encoding="utf-8")):
            for ch in literal:
                if ord(ch) > 0x7F:
                    cps.add(ord(ch))
    return cps


SELF_TEST_CASES = [
    ('let s = "渲染";', ["渲染"]),
    ('// 注释里的"引号"不算\nlet s = "真的";', ["真的"]),
    ('/* 块注释 "带引号" */ let s = "真的";', ["真的"]),
    ('/* 外层 /* 内层 "x" */ 仍在注释 */ let s = "真的";', ["真的"]),
    ('let s = r#"含 " 的原始串"#;', ['含 " 的原始串']),
    (r'let s = "转义 \" 不结束"; ', [r'转义 \" 不结束']),
    ("let c = '字'; let f: &'a str;", ["字"]),
    ('let s = "含 // 的字面量";', ["含 // 的字面量"]),
]


def self_test() -> int:
    """词法器的最小回归。它悄悄错掉的表现就是字体子集悄悄多字或少字。"""
    failures = 0
    for src, expect in SELF_TEST_CASES:
        got = [lit for lit in iter_literals(src) if lit]
        if got != expect:
            failures += 1
            print(f"[失败] {src!r}\n       期望 {expect!r}\n       实得 {got!r}")
    print("[通过] 词法器自检" if failures == 0 else f"{failures} 条词法自检失败")
    return failures


def load_source(path: Path, font_number: int):
    from fontTools.ttLib import TTFont

    font = TTFont(str(path), fontNumber=font_number) if path.suffix.lower() == ".ttc" \
        else TTFont(str(path))
    ps_name = font["name"].getDebugName(6)
    if ps_name != EXPECT_PS_NAME:
        raise SystemExit(
            f"源字体是 {ps_name}，期望 {EXPECT_PS_NAME}。"
            f"ttc 的子字体序号可能变了，用 --font-number 指定，或用 --expect 覆盖期望值。"
        )
    return font


def outline(font, cp: int):
    """码点的轮廓记录。用来判断两份字体在这个码点上是不是同一个字形。"""
    from fontTools.pens.recordingPen import RecordingPen

    cmap = font.getBestCmap()
    if cp not in cmap:
        return None
    pen = RecordingPen()
    font.getGlyphSet()[cmap[cp]].draw(pen)
    return repr(pen.value)


def verify(subset_path: Path, source: Path, font_number: int) -> int:
    """三条判据：文案覆盖、ASCII 覆盖、SC/JP 字形分支。返回失败条数。"""
    from fontTools.ttLib import TTCollection, TTFont

    failures = self_test()
    want = collect_codepoints()
    sub = TTFont(str(subset_path))
    have = set(sub.getBestCmap().keys())

    missing = sorted(want - have)
    if missing:
        failures += 1
        shown = "".join(chr(c) for c in missing[:60])
        print(f"[失败] 子集缺 {len(missing)} 个码点，前若干个: {shown}")
    else:
        print(f"[通过] 源码字面量所需的 {len(want)} 个码点全部在 cmap 里")

    ascii_missing = [c for c in range(0x20, 0x7F) if c not in have]
    if ascii_missing:
        failures += 1
        print(f"[失败] 可打印 ASCII 缺 {len(ascii_missing)} 个")
    else:
        print("[通过] 可打印 ASCII 全覆盖")

    # SC/JP 字形分支核对。只比较"SC 与 JP 确实不同"的码点：其余码点两支一模一样，
    # 比了也区分不出取的是哪一支。
    if not source.exists():
        print(f"[跳过] 找不到源字体 {source}，无法核对 SC/JP 字形分支")
        return failures

    if source.suffix.lower() == ".ttc":
        fonts = TTCollection(str(source)).fonts
        sc = fonts[font_number]
        jp = next(
            (f for f in fonts if f["name"].getDebugName(6) == "NotoSansCJKjp-Regular"), None
        )
    else:
        sc, jp = TTFont(str(source)), None

    if jp is None:
        print("[跳过] 源文件里没有 JP 分支，无法做字形分支核对")
        return failures

    differing = []
    for cp in sorted(have):
        if cp < 0x2E80:  # 只看 CJK 区；标点/拉丁两支通常一致
            continue
        a, b = outline(sc, cp), outline(jp, cp)
        if a is not None and b is not None and a != b:
            differing.append(cp)

    match_sc = sum(1 for cp in differing if outline(sub, cp) == outline(sc, cp))
    match_jp = sum(1 for cp in differing if outline(sub, cp) == outline(jp, cp))
    print(
        f"字形分支: SC/JP 形状不同的码点 {len(differing)} 个，"
        f"子集与 SC 一致 {match_sc} 个、与 JP 一致 {match_jp} 个"
    )
    if not differing:
        print("[跳过] 子集里没有 SC/JP 形状不同的码点，这一条无法判定")
    elif match_sc == len(differing) and match_jp == 0:
        print("[通过] 子集用的是 SC 字形分支")
    else:
        failures += 1
        print("[失败] 子集不是纯 SC 字形分支（很可能误取了 ttc 的 0 号 = JP）")

    name = sub["name"]
    print(f"子集 name 表: {name.getDebugName(6)} | {name.getDebugName(5)}")
    return failures


def build(subset_path: Path, source: Path, font_number: int) -> None:
    from fontTools.subset import Options, Subsetter

    font = load_source(source, font_number)
    cps = collect_codepoints()

    options = Options()
    # 保留 name/OS2 表里的版权与版本串：子集文件自己就是许可与来源的凭据，
    # 不依赖仓库里的说明文件。
    options.name_IDs = ["*"]
    options.name_legacy = True
    options.name_languages = ["*"]
    options.notdef_outline = True   # 缺字框要能画出来，否则漏字表现为"什么都没有"
    options.layout_features = ["*"]
    options.drop_tables = []
    options.recalc_bounds = True

    subsetter = Subsetter(options=options)
    subsetter.populate(unicodes=cps)
    subsetter.subset(font)
    font.save(str(subset_path))
    print(f"已写出 {subset_path}（{subset_path.stat().st_size} 字节，{len(cps)} 个码点）")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify", action="store_true", help="只校验，不改动子集文件")
    parser.add_argument("--self-test", action="store_true", help="只跑字面量词法器自检")
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--font-number", type=int, default=DEFAULT_FONT_NUMBER)
    parser.add_argument("--out", type=Path, default=OUT)
    args = parser.parse_args()

    if args.self_test:
        return 1 if self_test() else 0

    if args.verify:
        if not args.out.exists():
            print(f"[失败] 子集文件不存在: {args.out}")
            return 1
        failures = verify(args.out, args.source, args.font_number)
        print(f"\n{'全部通过' if failures == 0 else f'{failures} 条判据失败'}")
        return 0 if failures == 0 else 1

    if not args.source.exists():
        print(f"[失败] 源字体不存在: {args.source}（安装 fonts-noto-cjk，或用 --source 指定）")
        return 2
    build(args.out, args.source, args.font_number)
    return verify(args.out, args.source, args.font_number) and 1 or 0


if __name__ == "__main__":
    sys.exit(main())
