#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RViz 插件启动器（PyQt5 图形界面版，橙白扁平风格）"""

import os
import socket
import subprocess
import sys
from pathlib import Path

from PyQt5.QtCore import QRect, Qt
from PyQt5.QtGui import QColor, QFont, QFontMetrics
from PyQt5.QtWidgets import (
    QApplication, QHBoxLayout, QLabel, QListWidget, QListWidgetItem,
    QMessageBox, QStyle, QStyleOptionViewItem, QStyledItemDelegate,
    QTextBrowser, QVBoxLayout, QWidget,
)

SCRIPT_DIR = Path(__file__).resolve().parent
LIB_DIR = SCRIPT_DIR / "lib"
TARGET = LIB_DIR / "libmy_rviz_plugin.so"   # 软链目标名（rviz 加载用）
LAUNCH_DIR = SCRIPT_DIR / "launch"          # package.xml / rviz 配置所在目录
CONFIG = LAUNCH_DIR / "config.rviz"

ORANGE = "#FFA449"          # 主题色（橙白扁平）
ORANGE_DARK = "#FF975F"
ORANGE_LIGHT = "#FFF3E0"
TEXT_MAIN = "#333333"       # 列表主行
TEXT_SUB = "#FF6600"        # 列表副行（弱化备注）
TEXT_WARN = "#E65100"       # 副行提示强调色


# ---------------- Readme 解析 ----------------
def parse_readme(path: Path = SCRIPT_DIR / "Readme.md") -> dict:
    """解析 Readme.md：文件名 -> {'project','remark','topics','order'}"""
    info, order, project, cur = {}, 0, None, None
    if not path.exists():
        return info
    for line in (raw.strip() for raw in path.read_text(encoding="utf-8").splitlines()):
        if line.startswith("###"):                        # 章节标题 → 项目名
            project = line.lstrip("# ").strip()
        elif line.startswith("- lib/"):                   # 插件条目（可带备注）
            name = line[len("- lib/"):].split()[0]
            remark = (line.split("**(", 1)[1].rsplit(")**", 1)[0]
                      if "**(" in line else "")
            cur = {"project": project or "其它", "remark": remark,
                   "topics": [], "order": order}
            info[name], order = cur, order + 1
        elif cur is not None and line.startswith("- /"):  # 条目下 topic
            cur["topics"].append(line[2:].strip())
    return info


SO_INFO = parse_readme()   # 插件元信息以 Readme 为准


def project_of(fname: str) -> str:     # 未登记的归入“其它”
    return SO_INFO.get(fname, {}).get("project", "其它")


def remark_of(fname: str) -> str:      # 版本备注（如 批量回灌/8ms递进）
    return SO_INFO.get(fname, {}).get("remark", "")


def is_8ms(remark: str) -> bool:       # 是否非逐帧（8ms 区间递进）
    return "8ms" in remark or "不是逐帧" in remark


def collect_sos() -> list:
    """收集 lib 下所有真实 .so 文件（排除软链接）"""
    return (sorted(p for p in LIB_DIR.glob("*.so") if not p.is_symlink())
            if LIB_DIR.is_dir() else [])


def ros_master_ok(host="localhost", port=11311, timeout=0.5) -> bool:
    """检查 ROS Master (11311 端口) 是否在运行"""
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def make_env() -> dict:
    """构造子进程环境：叠加本目录路径到 ROS 相关环境变量"""
    env = os.environ.copy()
    for key, val in (("LD_LIBRARY_PATH", LIB_DIR),
                     ("ROS_PACKAGE_PATH", LAUNCH_DIR),
                     ("CMAKE_PREFIX_PATH", SCRIPT_DIR)):
        env[key] = str(val) + os.pathsep + env.get(key, "")
    return env


# 橙白扁平 QSS：白底黑字，选中项橙色反白，行间留白，圆角高亮
QSS = f"""
QWidget {{ background: white; color: black; font-size: 14px; }}
#titleBar {{ background: {ORANGE}; }}
QListWidget {{
    background: white; border: 1px solid #FFB74D; border-radius: 6px;
    padding: 6px; outline: none;
}}
QListWidget::item {{ border-radius: 6px; }}
QListWidget::item:hover {{ background: {ORANGE_LIGHT}; }}
QListWidget::item:selected {{ background: {ORANGE}; }}
#detailPanel {{
    background: {ORANGE_LIGHT};
    border: 1px solid #FFB74D; border-radius: 6px;
}}
QTextBrowser {{ background: transparent; border: none; font-size: 13px; }}
#keys {{ color: #888888; font-size: 12px; padding: 4px 0 0 0; }}
"""


ROLE_SUB = Qt.UserRole        # 副行普通文字（灰色弱化）
ROLE_WARN = Qt.UserRole + 1   # 副行提示文字（暖橙强调）


class RowDelegate(QStyledItemDelegate):
    """列表项自绘：主行正常色，副行小号弱化、提示段暖橙，选中随橙色背景反白"""

    def sizeHint(self, option, index):
        hint = super().sizeHint(option, index)
        hint.setHeight(56 if (index.data(ROLE_SUB) or index.data(ROLE_WARN)) else 34)
        return hint

    def paint(self, painter, option, index):
        opts = QStyleOptionViewItem(option)
        self.initStyleOption(opts, index)
        sub, warn = index.data(ROLE_SUB) or "", index.data(ROLE_WARN) or ""
        main, opts.text = opts.text, ""   # 文字自绘，背景交给样式绘制
        (opts.widget.style() if opts.widget else QApplication.style()
         ).drawControl(QStyle.CE_ItemViewItem, opts, painter, opts.widget)

        rect = opts.rect.adjusted(10, 0, -10, 0)
        selected = bool(opts.state & QStyle.State_Selected)
        align = int(Qt.AlignVCenter | Qt.AlignLeft)   # 显式 int，消除警告

        sub_font = QFont(opts.font)      # 副行比主行小两号
        if sub_font.pixelSize() > 0:
            sub_font.setPixelSize(max(sub_font.pixelSize() - 2, 9))
        elif sub_font.pointSizeF() > 0:
            sub_font.setPointSizeF(max(sub_font.pointSizeF() - 2, 8.0))
        fm, sfm = QFontMetrics(opts.font), QFontMetrics(sub_font)
        main = fm.elidedText(main, Qt.ElideMiddle, rect.width())

        painter.save()
        painter.setFont(opts.font)
        painter.setPen(QColor("white") if selected else QColor(TEXT_MAIN))
        if not (sub or warn):            # 单行：垂直居中
            painter.drawText(rect, align, main)
        else:                            # 双行：主行上，副行下
            top = rect.top() + max((rect.height() - fm.height() - 3 - sfm.height()) // 2, 2)
            painter.drawText(QRect(rect.left(), top, rect.width(), fm.height()), align, main)
            y = top + fm.height() + 3
            painter.setFont(sub_font)
            if selected:                 # 选中：整条副行浅橙
                painter.setPen(QColor("#FFE9CE"))
                whole = sub + ("，" + warn if sub and warn else warn)
                painter.drawText(QRect(rect.left(), y, rect.width(), sfm.height()), align,
                                 sfm.elidedText(whole, Qt.ElideRight, rect.width()))
            else:                        # 未选中：灰备注 + 暖橙提示 分段着色
                x = rect.left()
                if sub:
                    painter.setPen(QColor(TEXT_SUB))
                    text = sfm.elidedText(sub, Qt.ElideRight, rect.width())
                    painter.drawText(QRect(x, y, rect.width(), sfm.height()), align, text)
                    x += sfm.horizontalAdvance(text)
                if warn:
                    text = ("，" if sub else "") + warn
                    painter.setPen(QColor(TEXT_WARN))
                    remain = max(rect.right() - x, 1)
                    painter.drawText(QRect(x, y, remain, sfm.height()), align,
                                     sfm.elidedText(text, Qt.ElideRight, remain))
        painter.restore()


class Launcher(QWidget):
    """两级选择主窗口：左侧列表（项目→.so），右侧详情面板供二次确认"""

    def __init__(self):
        super().__init__()
        self.projects, self.stage, self.displayed, self.selected = {}, "project", [], None
        self.init_ui()

    def init_ui(self) -> None:
        self.setWindowTitle(" RVIZ 插件的 Release 发行版本")
        self.resize(880, 520)

        root = QVBoxLayout(self)
        root.setContentsMargins(14, 0, 14, 12)
        root.setSpacing(10)

        title = QLabel(
            "<span style='color:white;font-size:17px;font-weight:bold;'>AutoSar Radar RViz 插件启动器</span>"
            f"<span style='color:{ORANGE_LIGHT};font-size:13px;'>&nbsp;&nbsp;详见 Readme.md</span>")
        title.setObjectName("titleBar")
        title.setFixedHeight(46)
        title.setAlignment(Qt.AlignCenter)
        root.addWidget(title)

        self.tip = QLabel()
        root.addWidget(self.tip)

        body = QHBoxLayout()
        body.setSpacing(12)
        self.listw = QListWidget()
        self.listw.setItemDelegate(RowDelegate(self.listw))
        self.listw.setSpacing(8)            # 行与行之间留出间隔
        self.listw.setMouseTracking(True)
        self.listw.itemSelectionChanged.connect(self.on_select)
        body.addWidget(self.listw, 12)

        side = QVBoxLayout()
        side.setSpacing(8)
        self.detail_title = QLabel("详情")
        self.detail_title.setStyleSheet(f"color:{ORANGE_DARK}; font-weight:bold;")
        side.addWidget(self.detail_title)
        self.detail = QTextBrowser()
        self.detail.setObjectName("detailPanel")
        side.addWidget(self.detail, 1)
        body.addLayout(side, 9)
        root.addLayout(body, 1)

        keys = QLabel(" ↑/↓ 选择     Enter 进入/启动     Esc 返回上级 ")
        keys.setObjectName("keys")
        keys.setAlignment(Qt.AlignCenter)
        root.addWidget(keys)

    # ---------- 页面切换 ----------
    def _fill_list(self, rows) -> None:
        """重建列表；rows 为 (主行, 副行备注, 副行提示) 三元组"""
        self.listw.blockSignals(True)
        self.listw.clear()
        for main, sub, warn in rows:
            item = QListWidgetItem(main)
            if sub:
                item.setData(ROLE_SUB, sub)
            if warn:
                item.setData(ROLE_WARN, warn)
            self.listw.addItem(item)
        if rows:
            self.listw.setCurrentRow(0)
        self.listw.blockSignals(False)
        self.on_select()   # 手动刷新一次详情面板

    def _set_tip(self, step: int, text: str) -> None:
        """顶部提示行：步骤计数弱化为小号灰字，正文平缓表述"""
        self.tip.setText(f"<span style='color:#B9B9B9;font-size:12px;'>第 {step} 步，共 2 步</span>"
                         f"<span style='color:#555555;'>　{text}</span>")

    def _group_has_8ms(self, name: str) -> bool:
        return any(is_8ms(remark_of(s.name)) for s in self.projects[name])

    def show_projects(self) -> None:
        """一级页面：项目列表"""
        self.stage = "project"
        self.displayed = names = sorted(self.projects)
        rows = [(name, f"共 {len(self.projects[name])} 个版本",
                 "含非逐帧，区间递进版本" if self._group_has_8ms(name) else "")
                for name in names]
        self._fill_list(rows)
        self._set_tip(1, "请选择项目，按回车进入")

    def show_sos(self, project: str) -> None:
        """二级页面：该项目下的 .so 版本列表（顺序与 Readme 一致）"""
        self.stage = "so"
        items = sorted(self.projects[project],
                       key=lambda p: SO_INFO.get(p.name, {}).get("order", 999))
        self.displayed = items
        rows = []
        for so in items:
            remark = remark_of(so.name)
            rows.append((so.name, "" if is_8ms(remark) else remark,
                         remark if is_8ms(remark) else ""))  # 8ms 版本整条备注暖橙
        self._fill_list(rows)
        self._set_tip(2, f"当前项目 {project}，请选择插件版本，按回车启动")

    # ---------- 详情与交互 ----------
    def on_select(self) -> None:
        """选中变化时刷新右侧详情面板"""
        row = self.listw.currentRow()
        if not (0 <= row < len(self.displayed)):
            self.detail.setPlainText("")
            return
        val = self.displayed[row]
        if self.stage == "project":
            items = self.projects[val]
            warn = ("<br><b style='color:#E65100'>含非逐帧，区间递进版本</b>"
                    if self._group_has_8ms(val) else "")
            self.detail_title.setText(f"项目：{val}")
            self.detail.setHtml(f"<b>共 {len(items)} 个版本</b>{warn}<ul>"
                                f"{''.join(f'<li>{s.name}</li>' for s in items)}</ul>")
        else:
            remark = remark_of(val.name)
            topics = SO_INFO.get(val.name, {}).get("topics", [])
            remark_html = f"<br><b style='color:#E65100'>{remark}</b>" if remark else ""
            self.detail_title.setText("即将启用以下插件")
            self.detail.setHtml(
                f"<code>{val.name}</code>{remark_html}<br><b>----</b><br>"
                + (f"<br><b>发布 topic：</b><ul style='margin:2px'>"
                   f"{''.join(f'<li>{t}</li>' for t in topics)}</ul>" if topics else "")
                + "<br><span style='color:#888888'>按回车启动 RViz</span>")

    def on_activate(self) -> None:
        """回车：一级进入项目；二级确认启动"""
        row = self.listw.currentRow()
        if 0 <= row < len(self.displayed):
            if self.stage == "project":
                self.show_sos(self.displayed[row])
            else:
                self.accept(self.displayed[row])

    def action_back(self) -> None:
        if self.stage == "so":     # 返回一级项目列表
            self.show_projects()

    def keyPressEvent(self, event) -> None:
        key, mods = event.key(), event.modifiers()
        if key in (Qt.Key_Return, Qt.Key_Enter):
            self.on_activate()
        elif key == Qt.Key_Escape:
            self.action_back()
        else:
            super().keyPressEvent(event)

    def accept(self, selected: Path) -> None:
        """确认选择：记录后关闭窗口，由 main 继续软链并启动 rviz"""
        self.selected = selected
        self.close()


def main() -> None:
    app = QApplication(sys.argv)
    app.setFont(QFont("Noto Sans CJK SC", 11))
    app.setStyleSheet(QSS)

    if not ros_master_ok():   # 预检 ROS Master，未运行弹窗退出
        QMessageBox.critical(None, "ROS Master 未运行",
                             "未检测到 ROS Master")
        sys.exit(1)

    win = Launcher()          # 收集 .so 并按项目分组
    for so in collect_sos():
        # 实际存在的 .so 文件 → 查询 SO_INFO 字典 → 有定义显示信息，无定义归入"其它"
        win.projects.setdefault(project_of(so.name), []).append(so)
    if not win.projects:
        QMessageBox.critical(None, "错误", f"{LIB_DIR} 下没有可用的 .so 文件")
        sys.exit(1)
    if len(win.projects) == 1:   # 只有一个项目时直接进二级页
        win.show_sos(next(iter(win.projects)))
    else:
        win.show_projects()
    win.show()

    app.exec_()
    if win.selected is None:  # 未选择即关闭窗口
        sys.exit(0)

    if TARGET.is_symlink():   # 更新软链
        TARGET.unlink()
    TARGET.symlink_to(win.selected)

    env = make_env()
    print(f"已选择: {win.selected}")
    print("插件注册:")
    subprocess.run(["rospack", "plugins", "--attrib=plugin", "rviz"],
                   env=env, check=False)
    print("正在启动 AutoSar Radar RViz 插件...")
    sys.exit(subprocess.call(["rviz", "-d", str(CONFIG)], env=env))


if __name__ == "__main__":
    main()