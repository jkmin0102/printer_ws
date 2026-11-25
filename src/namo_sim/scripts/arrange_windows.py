#!/usr/bin/env python3
"""Tile only the existing demo windows and report a verified capture rectangle.

No window is created and no screenshot is taken. Run before ffmpeg, and use the
reported capture coordinates only when ready=true. Ambiguous titles, geometry
failure, or another managed window over the capture region cause a failure.
"""

import argparse
import ctypes as C
import ctypes.util
import json
import os
from pathlib import Path
import sys
import time


Window = C.c_ulong
Atom = C.c_ulong
Display = C.c_void_p


class Attributes(C.Structure):
    _fields_ = [
        ('x', C.c_int), ('y', C.c_int), ('width', C.c_int), ('height', C.c_int),
        ('border_width', C.c_int), ('depth', C.c_int), ('visual', C.c_void_p),
        ('root', Window), ('window_class', C.c_int), ('bit_gravity', C.c_int),
        ('win_gravity', C.c_int), ('backing_store', C.c_int),
        ('backing_planes', C.c_ulong), ('backing_pixel', C.c_ulong),
        ('save_under', C.c_int), ('colormap', C.c_ulong), ('map_installed', C.c_int),
        ('map_state', C.c_int), ('all_event_masks', C.c_long),
        ('your_event_mask', C.c_long), ('do_not_propagate_mask', C.c_long),
        ('override_redirect', C.c_int), ('screen', C.c_void_p),
    ]


class MessageData(C.Union):
    _fields_ = [('b', C.c_char * 20), ('s', C.c_short * 10), ('l', C.c_long * 5)]


class ClientMessage(C.Structure):
    _fields_ = [
        ('type', C.c_int), ('serial', C.c_ulong), ('send_event', C.c_int),
        ('display', Display), ('window', Window), ('message_type', Atom),
        ('format', C.c_int), ('data', MessageData),
    ]


class Event(C.Union):
    _fields_ = [('xclient', ClientMessage), ('pad', C.c_long * 24)]


class X11:
    def __init__(self, display_name):
        library = ctypes.util.find_library('X11')
        if not library:
            raise RuntimeError('libX11 is unavailable')
        self.lib = C.CDLL(library)
        signatures = {
            'XOpenDisplay': ([C.c_char_p], Display),
            'XCloseDisplay': ([Display], C.c_int),
            'XDefaultRootWindow': ([Display], Window),
            'XInternAtom': ([Display, C.c_char_p, C.c_int], Atom),
            'XGetAtomName': ([Display, Atom], C.c_void_p),
            'XGetWindowProperty': ([Display, Window, Atom, C.c_long, C.c_long,
                                   C.c_int, Atom, C.POINTER(Atom), C.POINTER(C.c_int),
                                   C.POINTER(C.c_ulong), C.POINTER(C.c_ulong),
                                   C.POINTER(C.c_void_p)], C.c_int),
            'XFetchName': ([Display, Window, C.POINTER(C.c_void_p)], C.c_int),
            'XFree': ([C.c_void_p], C.c_int),
            'XGetWindowAttributes': ([Display, Window, C.POINTER(Attributes)], C.c_int),
            'XTranslateCoordinates': ([Display, Window, Window, C.c_int, C.c_int,
                                       C.POINTER(C.c_int), C.POINTER(C.c_int),
                                       C.POINTER(Window)], C.c_int),
            'XSendEvent': ([Display, Window, C.c_int, C.c_long, C.POINTER(Event)], C.c_int),
            'XRaiseWindow': ([Display, Window], C.c_int),
            'XFlush': ([Display], C.c_int),
            'XSync': ([Display, C.c_int], C.c_int),
            'XSetErrorHandler': ([C.c_void_p], C.c_void_p),
        }
        for name, (args, result) in signatures.items():
            method = getattr(self.lib, name)
            method.argtypes, method.restype = args, result
        self.display = self.lib.XOpenDisplay(display_name.encode() if display_name else None)
        if not self.display:
            raise RuntimeError('Cannot open X display; arrange_windows requires desktop access')
        self.root = self.lib.XDefaultRootWindow(self.display)
        self.atoms = {}
        # Splash windows can be destroyed between discovery and a property
        # read. Preserve normal Python timeout/error handling rather than let
        # Xlib's default handler terminate the process without JSON output.
        error_callback = C.CFUNCTYPE(C.c_int, Display, C.c_void_p)
        self.error_handler = error_callback(lambda _display, _event: 0)
        self.previous_error_handler = self.lib.XSetErrorHandler(
            C.cast(self.error_handler, C.c_void_p))

    def atom(self, name):
        if name not in self.atoms:
            self.atoms[name] = self.lib.XInternAtom(self.display, name.encode(), False)
        return self.atoms[name]

    def property(self, window, name):
        actual, fmt, count, remaining = Atom(), C.c_int(), C.c_ulong(), C.c_ulong()
        pointer = C.c_void_p()
        status = self.lib.XGetWindowProperty(
            self.display, window, self.atom(name), 0, 8192, False, 0,
            C.byref(actual), C.byref(fmt), C.byref(count), C.byref(remaining),
            C.byref(pointer))
        if status != 0 or not pointer.value:
            return []
        try:
            if fmt.value == 32:
                values = C.cast(pointer, C.POINTER(C.c_ulong))
                return [int(values[i]) for i in range(count.value)]
            if fmt.value == 8:
                return C.string_at(pointer, count.value).decode('utf-8', errors='replace')
            return []
        finally:
            self.lib.XFree(pointer)

    def atom_name(self, atom):
        pointer = self.lib.XGetAtomName(self.display, atom)
        if not pointer:
            return str(atom)
        try:
            return C.string_at(pointer).decode('utf-8', errors='replace')
        finally:
            self.lib.XFree(pointer)

    def title(self, window):
        title = self.property(window, '_NET_WM_NAME')
        if isinstance(title, str) and title:
            return title
        pointer = C.c_void_p()
        if self.lib.XFetchName(self.display, window, C.byref(pointer)) and pointer.value:
            try:
                return C.string_at(pointer).decode('utf-8', errors='replace')
            finally:
                self.lib.XFree(pointer)
        return ''

    def attributes(self, window):
        attr = Attributes()
        if not self.lib.XGetWindowAttributes(self.display, window, C.byref(attr)):
            raise RuntimeError(f'Window {window} disappeared')
        return attr

    def rectangle(self, window):
        attr = self.attributes(window)
        x, y, child = C.c_int(), C.c_int(), Window()
        if not self.lib.XTranslateCoordinates(self.display, window, self.root, 0, 0,
                                              C.byref(x), C.byref(y), C.byref(child)):
            raise RuntimeError('Cannot translate window coordinates to screen')
        extents = self.property(window, '_NET_FRAME_EXTENTS')
        if len(extents) != 4:
            extents = [0, 0, 0, 0]
        left, right, top, bottom = extents
        return [x.value-left, y.value-top, attr.width+left+right,
                attr.height+top+bottom]

    def send(self, window, message, values):
        event = Event()
        event.xclient.type = 33  # ClientMessage
        event.xclient.display = self.display
        event.xclient.window = window
        event.xclient.message_type = self.atom(message)
        event.xclient.format = 32
        for i, value in enumerate(values):
            event.xclient.data.l[i] = value
        mask = (1 << 20) | (1 << 19)  # SubstructureRedirect | SubstructureNotify
        if not self.lib.XSendEvent(self.display, self.root, False, mask, C.byref(event)):
            raise RuntimeError(f'Window manager rejected {message}')
        self.lib.XFlush(self.display)

    def unmaximize(self, window):
        self.send(window, '_NET_WM_STATE', [
            0, self.atom('_NET_WM_STATE_MAXIMIZED_VERT'),
            self.atom('_NET_WM_STATE_MAXIMIZED_HORZ'), 2, 0])
        self.send(window, '_NET_WM_STATE', [0, self.atom('_NET_WM_STATE_FULLSCREEN'), 0, 2, 0])

    def keep_above(self, window):
        # Keep only these PID-verified demonstration windows above ordinary
        # editor windows when a tool approval temporarily changes keyboard focus.
        self.send(window, '_NET_WM_STATE', [1, self.atom('_NET_WM_STATE_ABOVE'), 0, 2, 0])

    def place(self, window, outer):
        extents = self.property(window, '_NET_FRAME_EXTENTS')
        left, right, top, bottom = extents if len(extents) == 4 else [0, 0, 0, 0]
        x, y, width, height = outer
        width, height = width-left-right, height-top-bottom
        if width <= 0 or height <= 0:
            raise RuntimeError('Invalid window frame extents')
        # Ask the window manager to raise its decorated frame, not merely the
        # client inside the frame. These requests target only selected demos.
        self.send(window, '_NET_ACTIVE_WINDOW', [2, 0, 0, 0, 0])
        self.send(window, '_NET_RESTACK_WINDOW', [2, 0, 0, 0, 0])  # Above
        # StaticGravity + all four geometry fields + application source id.
        self.send(window, '_NET_MOVERESIZE_WINDOW',
                  [10 | (15 << 8) | (1 << 12), x+left, y+top, width, height])
        self.lib.XRaiseWindow(self.display, window)
        self.lib.XFlush(self.display)

    def close(self):
        if self.display:
            self.lib.XCloseDisplay(self.display)
            self.display = None
            self.lib.XSetErrorHandler(self.previous_error_handler)


def overlap(a, b):
    return (a[0] < b[0]+b[2] and b[0] < a[0]+a[2]
            and a[1] < b[1]+b[3] and b[1] < a[1]+a[3])


def belongs_to_process(pid, required_pid):
    """Accept the launch process itself or a descendant that owns its Qt window."""
    visited = set()
    while pid and pid not in visited:
        if pid == required_pid:
            return True
        visited.add(pid)
        try:
            # comm can contain spaces and parentheses; fields after its final ')'
            # start with state and ppid. No process is signalled or modified.
            fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
            pid = int(fields[1])
        except (OSError, ValueError, IndexError):
            return False
    return False


def select_windows(x11, args):
    clients = x11.property(x11.root, '_NET_CLIENT_LIST')
    matches = {'gazebo': [], 'rviz': []}
    normal = x11.atom('_NET_WM_WINDOW_TYPE_NORMAL')
    excluded = {x11.atom('_NET_WM_WINDOW_TYPE_'+kind) for kind in (
        'SPLASH', 'DIALOG', 'UTILITY', 'TOOLBAR', 'DOCK', 'DESKTOP',
        'MENU', 'DROPDOWN_MENU', 'POPUP_MENU', 'TOOLTIP', 'NOTIFICATION',
    )}
    for window in clients:
        attr = x11.attributes(window)
        if (attr.map_state != 2 or attr.override_redirect
                or attr.width <= 320 or attr.height <= 320):
            continue
        window_types = set(x11.property(window, '_NET_WM_WINDOW_TYPE'))
        # An absent type is NORMAL under EWMH. In particular, do not mistake
        # Gazebo's mapped 576x320 splash for its main application window.
        if (window_types & excluded
                or (window_types and normal not in window_types)):
            continue
        title = x11.title(window)
        for kind, fragment, required_pid in (
                ('gazebo', args.gazebo_title, args.gazebo_pid),
                ('rviz', args.rviz_title, args.rviz_pid)):
            if fragment.casefold() not in title.casefold():
                continue
            pid = x11.property(window, '_NET_WM_PID')
            if (required_pid is not None
                    and (not pid or not belongs_to_process(pid[0], required_pid))):
                continue
            matches[kind].append((window, title, pid[0] if pid else None))
    for kind, windows in matches.items():
        if len(windows) > 1:
            raise RuntimeError(f'Ambiguous {kind} windows; provide --{kind}-pid or --{kind}-title')
    if not all(matches.values()):
        return None
    selected = {name: items[0] for name, items in matches.items()}
    if selected['gazebo'][0] == selected['rviz'][0]:
        raise RuntimeError('Gazebo and RViz matches refer to the same window')
    return selected


def verify_no_managed_overlap(x11, selected, rectangles):
    stacking = x11.property(x11.root, '_NET_CLIENT_LIST_STACKING')
    ids = [selected[name][0] for name in ('gazebo', 'rviz')]
    if not all(window in stacking for window in ids):
        raise RuntimeError('Window manager stacking information is unavailable')
    for index, window in enumerate(stacking):
        if window in ids or x11.attributes(window).map_state != 2:
            continue
        rect = x11.rectangle(window)
        for demo, demo_rect in zip(ids, rectangles):
            if index > stacking.index(demo) and overlap(rect, demo_rect):
                pid = x11.property(window, '_NET_WM_PID')
                detail = {
                    'id': window, 'title': x11.title(window),
                    'pid': pid[0] if pid else None, 'rectangle': rect,
                    'types': [x11.atom_name(atom) for atom in
                              x11.property(window, '_NET_WM_WINDOW_TYPE')],
                    'same_demo_process': bool(pid and any(
                        item[2] is not None and belongs_to_process(pid[0], item[2])
                        for item in selected.values())),
                }
                raise RuntimeError('Another window overlaps the demo: '+json.dumps(detail))


def selection_identity(selected):
    return tuple(selected[name] for name in ('gazebo', 'rviz')) if selected else None


def arrange(args):
    deadline = time.monotonic()+max(0.0, min(args.wait, 35.0))
    x11 = X11(args.display)
    try:
        screen = x11.attributes(x11.root)
        if screen.width < 1280 or screen.height < 720:
            raise RuntimeError(f'Display {screen.width}x{screen.height} is smaller than 1280x720')
        x = y = 0
        workareas = x11.property(x11.root, '_NET_WORKAREA')
        desktop = x11.property(x11.root, '_NET_CURRENT_DESKTOP')
        offset = 4*(desktop[0] if desktop else 0)
        if len(workareas) >= offset+4:
            area = workareas[offset:offset+4]
            x, y = max(0, area[0]), max(0, area[1])
        tile_width = 640
        rectangles = []
        identity = None
        stable_since = time.monotonic()
        clear_since = None
        unmaximized = set()
        attempts = 0
        last_reason = 'Both unique visible main demo windows did not appear within timeout'
        while time.monotonic() < deadline-0.4:
            try:
                selected = select_windows(x11, args)
            except RuntimeError as exc:
                last_reason = str(exc)
                selected = None
            current = selection_identity(selected)
            if current != identity:
                identity = current
                stable_since = time.monotonic()
                clear_since = None
                attempts = 0
            if not selected or time.monotonic()-stable_since < 3.0:
                time.sleep(0.2)
                continue
            for window, _, _ in selected.values():
                if window not in unmaximized:
                    x11.unmaximize(window)
                    x11.keep_above(window)
                    unmaximized.add(window)
            for index, kind in enumerate(('gazebo', 'rviz')):
                x11.place(selected[kind][0], [x+index*tile_width, y, tile_width, 720])
            time.sleep(0.3)
            x11.lib.XSync(x11.display, False)
            try:
                after = select_windows(x11, args)
            except RuntimeError as exc:
                last_reason = str(exc)
                after = None
            if selection_identity(after) != identity:
                identity = None
                clear_since = None
                continue
            try:
                rectangles = [x11.rectangle(selected[kind][0]) for kind in ('gazebo', 'rviz')]
            except RuntimeError as exc:
                last_reason = str(exc)
                identity = None
                clear_since = None
                continue
            left, right = rectangles
            if (left[2:] == [tile_width, 720] and right[2:] == [tile_width, 720]
                    and right[0] == left[0]+tile_width and right[1] == left[1]
                    and left[0] >= 0 and left[1] >= 0
                    and right[0]+tile_width <= screen.width and left[1]+720 <= screen.height):
                if any(x11.atom('_NET_WM_STATE_ABOVE') not in
                       x11.property(item[0], '_NET_WM_STATE')
                       for item in selected.values()):
                    last_reason = 'Window manager has not enabled ABOVE for both demo windows'
                    clear_since = None
                    continue
                try:
                    verify_no_managed_overlap(x11, selected, rectangles)
                except RuntimeError as exc:
                    last_reason = str(exc)
                    clear_since = None
                    # Startup splash/transient windows must actually disappear
                    # or be fully below both demos, even if their PID is ours.
                    continue
                if clear_since is None:
                    clear_since = time.monotonic()
                elif time.monotonic()-clear_since >= 1.0:
                    return {
                        'ready': True, 'display': args.display or os.environ.get('DISPLAY', ''),
                        'demo_windows_above': True,
                        'x': left[0], 'y': left[1], 'width': tile_width*2, 'height': 720,
                        'windows': {
                            kind: {'id': selected[kind][0], 'title': selected[kind][1],
                                   'pid': selected[kind][2], 'outer_rectangle': rectangles[index]}
                            for index, kind in enumerate(('gazebo', 'rviz'))
                        },
                    }
                continue
            clear_since = None
            last_reason = f'Could not tile exact adjacent {tile_width}x720 windows: {rectangles}'
            # Window managers may reserve a top panel and left dock. Rebase
            # both targets on the first actual outer origin, excluding panels
            # from capture and avoiding overlapping the second demo window.
            x = max(0, left[0])
            y = max(0, left[1], right[1])
            # A main window's minimum-size hint may prohibit 640 pixels. A
            # wider verified capture is allowed only when the screen fits it.
            attempts += 1
            if attempts >= 4 and max(left[2], right[2]) > tile_width:
                proposed = max(800, ((max(left[2], right[2])+19)//20)*20)
                if x+proposed*2 <= screen.width:
                    tile_width = proposed
        raise RuntimeError(last_reason)
    finally:
        x11.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--display', default=os.environ.get('DISPLAY'))
    parser.add_argument('--gazebo-title', default='Gazebo')
    parser.add_argument('--rviz-title', default='namo.rviz')
    parser.add_argument('--gazebo-pid', type=int)
    parser.add_argument('--rviz-pid', type=int)
    parser.add_argument('--wait', type=float, default=35.0)
    args = parser.parse_args()
    try:
        result = arrange(args)
    except Exception as exc:
        print(json.dumps({'ready': False, 'reason': str(exc)}))
        return 1
    print(json.dumps(result))
    return 0


if __name__ == '__main__':
    sys.exit(main())
