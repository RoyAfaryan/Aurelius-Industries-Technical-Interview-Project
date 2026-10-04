"""Visual language of the C2 station: black, white and gray, with red reserved for danger.

Rules:
  * Brightness means attention. Settled information is gray, live data is
    white, and anything that needs the operator's attention is an inverted
    white chip.
  * Red appears only for danger: link lost, a failed command, critical battery.
  * Sharp corners and 1 px hairlines; structure comes from lines, not fills.
"""

import sys
from dataclasses import dataclass

import customtkinter as ctk

# --- Color tokens -------------------------------------------------------
BLACK = "#000000"   # window and panel background
RAISED = "#161616"  # hover, dropdowns
LINE = "#262626"    # hairlines between regions
EDGE = "#4D4D4D"    # control outlines, disabled text
GRAY = "#8C8C8C"    # labels and settled information
WHITE = "#F5F5F5"   # live data and primary text
DANGER = "#E5484D"  # the only accent: danger and failure
NOMINAL = "#3FB950"  # green: the link status light only

# --- Semantic styles: (text color, background) --------------------------
# Pure helpers return a style name; the UI turns it into colors here.
STYLES = {
    "normal": (WHITE, "transparent"),
    "quiet": (GRAY, "transparent"),
    "attention": (BLACK, WHITE),      # inverted chip
    "danger": (DANGER, "transparent"),
}

# Color of the link status light for each style
DOT_COLORS = {"normal": NOMINAL, "quiet": GRAY, "attention": WHITE, "danger": DANGER}

@dataclass(frozen=True)
class Fonts:
    wordmark: ctk.CTkFont
    data: ctk.CTkFont
    label: ctk.CTkFont
    section: ctk.CTkFont
    button: ctk.CTkFont
    button_bold: ctk.CTkFont
    log: ctk.CTkFont


def make_fonts() -> Fonts:
    """Create fonts. Must be called after the root window exists."""
    if sys.platform == "win32":
        data, ui, mono = "Bahnschrift", "Segoe UI", "Consolas"
    elif sys.platform == "darwin":
        data, ui, mono = "DIN Alternate", "Helvetica Neue", "Menlo"
    else:
        data, ui, mono = "DejaVu Sans", "DejaVu Sans", "DejaVu Sans Mono"
    return Fonts(
        wordmark=ctk.CTkFont(family=data, size=19, weight="bold"),
        data=ctk.CTkFont(family=data, size=16),
        label=ctk.CTkFont(family=ui, size=12),
        section=ctk.CTkFont(family=ui, size=13, weight="bold"),
        button=ctk.CTkFont(family=ui, size=13),
        button_bold=ctk.CTkFont(family=ui, size=14, weight="bold"),
        log=ctk.CTkFont(family=mono, size=12),
    )


# --- Widget factories: every control gets the same sharp, outlined look ---

def panel(parent, **kwargs) -> ctk.CTkFrame:
    """A region outlined with a hairline."""
    return ctk.CTkFrame(parent, fg_color=BLACK, corner_radius=0, border_width=1, border_color=LINE, **kwargs)


def button(parent, text: str, command, fonts: Fonts, *, primary: bool = False, width: int = 0) -> ctk.CTkButton:
    """Outlined button; primary=True gives the single inverted, filled style."""
    if primary:
        colors = dict(fg_color=WHITE, hover_color="#CFCFCF", text_color=BLACK,
                      border_width=0, font=fonts.button_bold)
    else:
        colors = dict(fg_color=BLACK, hover_color=RAISED, text_color=WHITE,
                      border_width=1, border_color=EDGE, font=fonts.button)
    widget = ctk.CTkButton(parent, text=text, command=command, corner_radius=0, height=32,
                           text_color_disabled=EDGE, width=width or 140, **colors)
    widget.is_primary = primary
    return widget


def entry(parent, fonts: Fonts, **kwargs) -> ctk.CTkEntry:
    return ctk.CTkEntry(parent, corner_radius=0, fg_color=BLACK, border_width=1, border_color=EDGE,
                        text_color=WHITE, font=fonts.label, height=32, **kwargs)


def option_menu(parent, values: list[str], fonts: Fonts) -> ctk.CTkOptionMenu:
    return ctk.CTkOptionMenu(parent, values=values, corner_radius=0, height=32, font=fonts.button,
                             fg_color=RAISED, button_color=RAISED, button_hover_color=LINE,
                             text_color=WHITE, text_color_disabled=EDGE,
                             dropdown_fg_color=RAISED, dropdown_hover_color=LINE,
                             dropdown_text_color=WHITE, dropdown_font=fonts.button)


def slider(parent, **kwargs) -> ctk.CTkSlider:
    return ctk.CTkSlider(parent, corner_radius=0, height=18,
                         fg_color=LINE, progress_color=WHITE,
                         button_color=WHITE, button_hover_color="#CFCFCF", **kwargs)


def set_enabled(widget, enabled: bool) -> None:
    """Enable or disable a control so that its disabled state is visibly dimmed."""
    widget.configure(state="normal" if enabled else "disabled")
    if isinstance(widget, ctk.CTkEntry):
        widget.configure(text_color=WHITE if enabled else EDGE)
    elif isinstance(widget, ctk.CTkButton) and getattr(widget, "is_primary", False):
        # A filled white button looks active even when disabled, so dim its fill too
        widget.configure(fg_color=WHITE if enabled else LINE)