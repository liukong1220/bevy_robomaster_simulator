# assets/fonts/hud_cjk.otf

HUD 用中文字体。bevy 内置默认字体只有拉丁字形，中文会整段渲染成缺字框，所以必须自带
一份带 CJK 字形的字体；而 CJK 全字库有 19MB 量级，因此入库的是**按源码里实际出现的
字符生成的子集**（当前 289 个码点 / 139476 字节，2026-09-01 实测）。

## 来源与版本

| 项 | 值 |
| --- | --- |
| 字族 | Noto Sans CJK **SC**（简体中文字形分支） |
| PostScript 名 | `NotoSansCJKsc-Regular` |
| 版本 | `Version 2.004;hotconv 1.0.118;makeotfexe 2.5.65603` |
| 版权 | © 2014-2021 Adobe (http://www.adobe.com/) |
| 许可 | SIL Open Font License 1.1，全文见同目录 [`OFL.txt`](OFL.txt) |
| 本机源文件 | `/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc`，**子字体序号 2** |
| 源文件来源 | Debian/Ubuntu 包 `fonts-noto-cjk` 版本 `1:20220127+repack1-1` |

子集文件自身的 name 表保留了上述版本与版权串，可用
`python3 -c "from fontTools.ttLib import TTFont; n=TTFont('assets/fonts/hud_cjk.otf')['name']; print(n.getDebugName(5), n.getDebugName(6), n.getDebugName(0))"`
复核，不依赖本文件的记载。

### 为什么必须确认是 SC 而不是 JP

`NotoSansCJK-Regular.ttc` 里 10 个子字体共用同一批码点，但**同一个码点的字形不同**：
JP 分支用日本习惯字形（`NotoSansCJKjp-Regular` 是 ttc 的 0 号，pyftsubset 不带
`--font-number` 时取的就是它）。取错分支不会报错、字数也不会少，只是"会""来""令"
这些字长成日文写法，肉眼很容易漏过。

`scripts/build_hud_font.py --verify` 会做一次针对性核对：先找出子集码点里 SC 与 JP
形状**确实不同**的那些，再逐个比对子集用的是哪一支。最近一次运行的结果是
**86 个码点 SC/JP 形状不同，子集与 SC 全部一致、与 JP 零个一致**。这个数是"子集
里恰好落在两支形状不同的那些码点"的个数，会随字表增减而变，不是两支字体的差异总量。

## 重新生成

改动任何界面文案后必须重新生成，否则新增的字在 HUD 上是缺字框（□），而 Rust 编译期
完全不会提示。子集字表**必须**从源码扫描得到，不能手写：第一版手列字表漏了"数"
"方向键""空格"等等。

```sh
# 需要 fontTools（本机 4.29.1）：pip install fonttools
python3 scripts/build_hud_font.py            # 扫源码 -> 重建 assets/fonts/hud_cjk.otf
python3 scripts/build_hud_font.py --verify    # 只校验现有文件，不改动它（CI 用这条）
```

`--verify` 的判据有四条，任何一条不过就返回非零：
1. 字面量词法器自检（也可用 `--self-test` 单独跑）；
2. `src/` 下所有字符串字面量里的非 ASCII 字符全部在子集 cmap 里；
3. 可打印 ASCII 全部在 cmap 里；
4. 上面那条 SC/JP 字形分支核对。

## 扫描口径：严格忽略注释

字表只取**字符串与字符字面量**，注释一个字都不收。注释不会被渲染，收进子集只是白白
增大文件，还会把"改一行注释"变成"字体过期"。

这条口径以前只写在脚本注释里，实现却是拿正则扫原始文件文本：中文注释里出现一个引号
（很常见），从它到下一个引号之间的注释正文就被当成字符串字面量收走了。按那个口径扫
出来是 325 个码点，其中 **93 个只出现在注释里**（一、且、个、丸、之、事、于、交、件、
会……），HUD 永远不会渲染它们。现在脚本自己走一遍 Rust 词法：跳过行注释与可嵌套块
注释，正确处理原始字符串（`r#"含 " 的文本"#`）、转义与生命周期（`&'a str`），同一份
源码扫出 232 个码点。

那次对比用的是修口径当天的源码快照。字表随源码走：后来把 HUD 帮助行改成鼠标绑定
说明（左键开火 / 按住右键自瞄）又引入了一批新字，当前口径下是 289 个码点。所以
README 里的数字都带日期，改完字符串要重跑一次生成再更新这里。

词法器悄悄错掉的表现就是子集悄悄多字或少字，所以它自带最小回归：

```sh
python3 scripts/build_hud_font.py --self-test
```
