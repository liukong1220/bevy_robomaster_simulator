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
STRING_LITERALS = re.compile(
    r'''
      r\#*"(?:[^"]*)"\#*      # 原始字符串 r"..." / r#"..."#
    | "(?:\\.|[^"\\])*"       # 普通字符串
    | '(?:\\.|[^'\\])'        # 字符字面量
    ''',
    re.VERBOSE | re.DOTALL,
)


def collect_codepoints() -> set[int]:
    """src/ 下所有字符串/字符字面量里的非 ASCII 字符 + 全部可打印 ASCII。"""
    # 可打印 ASCII 无条件全收：HUD 会格式化数字、单位、方向键提示等等，
    # 按"源码里出现过"筛 ASCII 会因为一次 format! 拼接就漏字。
    cps = {c for c in range(0x20, 0x7F)}
    for path in sorted(SRC_DIR.rglob("*.rs")):
        text = path.read_text(encoding="utf-8")
        for m in STRING_LITERALS.finditer(text):
            for ch in m.group(0):
                if ord(ch) > 0x7F:
                    cps.add(ord(ch))
    return cps


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

    want = collect_codepoints()
    sub = TTFont(str(subset_path))
    have = set(sub.getBestCmap().keys())
    failures = 0

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
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--font-number", type=int, default=DEFAULT_FONT_NUMBER)
    parser.add_argument("--out", type=Path, default=OUT)
    args = parser.parse_args()

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
