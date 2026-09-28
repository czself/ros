"""车牌文本规范化与校验 —— 纯逻辑, 不依赖任何 OCR 库。

拆出来单独一个模块的原因: 这是最值得写单测的部分, 却不该被 paddleocr
的安装问题挡住。OCR 引擎换个后端, 这层规则不用动。

规则参考 GA 36-2018《机动车号牌〉。
"""
from __future__ import annotations

import re
import unicodedata
from dataclasses import dataclass
from typing import Optional

#: 省级行政区简称 + 特殊用途前缀
PROVINCE_CHARS = set(
    "京津冀晋蒙辽吉黑沪苏浙皖闽赣鲁豫鄂湘粤桂琼渝川贵云藏陕甘青宁新"
    "使领警学港澳"
)

#: 发牌机关代号 (地级市/地区), 第一批
CITY_CHARS = set("ABCDEFGHJKLMNPQRSTUVWXY")  # 不含 I, O

#: 分隔符不做特殊处理 —— normalize() 里的正则会直接剔除所有非中文/字母/数字字符,
#: 涵盖 · ・ • - _ 以及各种空白。


@dataclass(frozen=True)
class PlateText:
    """一次 OCR 的结果。"""

    raw: str                      #: OCR 原始输出, 原样保留便于排查
    text: str = ""                #: 规范化后的车牌 (无分隔符, 大写)
    display: str = ""             #: 带间隔号的展示形式, 如 "苏A·B8Q62"
    confidence: float = 0.0
    #: 校验失败的原因; None 表示通过
    issue: Optional[str] = None

    @property
    def is_valid(self) -> bool:
        return self.issue is None and bool(self.text)

    def __str__(self) -> str:
        return self.display or self.text or self.raw


def to_halfwidth(text: str) -> str:
    """全角 -> 半角。OCR 常把数字/字母识别成全角, 这是最常见的错误来源。"""
    out = []
    for ch in text:
        code = ord(ch)
        if code == 0x3000:  # 表意空格
            out.append(" ")
        elif 0xFF01 <= code <= 0xFF5E:
            out.append(chr(code - 0xFEE0))
        else:
            out.append(ch)
    return "".join(out)


def normalize(raw: str) -> str:
    """把 OCR 原始输出整理成规范车牌字符串 (纯大写字母+数字, 无分隔符)。

    >>> normalize("苏Ａ·B8Q62")
    '苏AB8Q62'
    >>> normalize(" 粤B 12345 ")
    '粤B12345'
    """
    if not raw:
        return ""
    # NFKC 会把全角字母数字、兼容字符一并折叠
    text = unicodedata.normalize("NFKC", raw)
    text = to_halfwidth(text)

    # 只保留中文、字母、数字
    text = re.sub(r"[^一-鿿A-Za-z0-9]", "", text)
    text = text.upper()

    # 形近字纠错只作用于序号段 (下标 >= 2), 前两位是省份 + 机关代号
    head, tail = text[:2], text[2:]
    return head + "".join(_fix_confusable(c) for c in tail)


#: GA 16735 规定正式车牌序号段不使用 I 和 O (为避免与 1、0 混淆), 因此这两个
#: 字母一旦出现基本可判定为误识, 直接映射回去比报错更有用。
#:
#: 注意: **L 是允许的**, 不能一并映射成 1 —— 真实车牌里 "苏A·PL12A" 这类很常见。
_SERIAL_LOOKALIKE = {"I": "1", "O": "0"}


def _fix_confusable(ch: str) -> str:
    return _SERIAL_LOOKALIKE.get(ch, ch)


def split_display(text: str) -> str:
    """插入间隔号: 省份 + 机关 + 其余。空串或长度不对就原样返回。

    >>> split_display("苏AB8Q62")
    '苏A·B8Q62'
    """
    if len(text) < 2:
        return text
    return f"{text[0]}{text[1]}·{text[2:]}"


def validate(text: str) -> Optional[str]:
    """返回 None 表示通过, 否则返回不合法的原因。

    放行: 7 位普通车牌, 8 位新能源车牌。
    另有 2~4 位的特殊号牌 (警用/ embassy 等), 这里只给 warning 级别,
    由调用方决定要不要收。
    """
    if not text:
        return "空结果"

    n = len(text)
    if n in (2, 3, 4):
        return f"长度 {n}, 疑似特殊号牌 (非标准民用车牌)"

    if n not in (7, 8):
        return f"长度 {n}, 期望 7 (普通) 或 8 (新能源)"

    if text[0] not in PROVINCE_CHARS:
        return f"首字 {text[0]!r} 不是有效的省份简称"

    if not ("A" <= text[1] <= "Z"):
        return f"第二位 {text[1]!r} 不是发牌机关代号字母"

    body = text[2:]
    if not body.isalnum():
        bad = [c for c in body if not c.isalnum()]
        return f"序号含非字母数字字符: {''.join(bad)!r}"

    # I 和 O 在正式车牌里不用, 出现基本可判定为 1/0 的误识
    for c in body:
        if c in "IO":
            return f"序号含 {c!r}, 正式车牌不使用 I/O, 疑似 1/0 误识"

    if n == 8 and not body[:5].isdigit() and not body[1:].isdigit():
        return "8 位新能源车牌序号格式可疑"

    return None


def is_new_energy(text: str) -> bool:
    """8 位且序号主要为数字 -> 判定为新能源车牌。"""
    return len(text) == 8 and sum(c.isdigit() for c in text[2:]) >= 5


def build(
    raw: str,
    confidence: float = 0.0,
    strict: bool = False,
) -> PlateText:
    """OCR 原始输出 -> 结构化结果。

    ``strict=True`` 时, 校验不通过的结果 **text 和 display 都清空** ——
    只清 text 是不够的: display 是给人看的字段, 且 ``__str__`` 优先返回它,
    脏车牌照样会漏出去。``raw`` 始终原样保留, 排查时还能看到引擎认出了什么。
    """
    text = normalize(raw)
    issue = validate(text)
    keep = not (strict and issue)
    return PlateText(
        raw=raw,
        text=text if keep else "",
        display=split_display(text) if (keep and text) else "",
        confidence=float(confidence),
        issue=issue,
    )


def format_for_file(display_or_text: str) -> str:
    """回写文本文件用的单行形式 (去掉制表符/换行, 保证一行一条)。"""
    return re.sub(r"[\t\r\n]", " ", display_or_text).strip()
