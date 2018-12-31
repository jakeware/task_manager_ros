import lorem
import urwid
import urwid.raw_display
import time
import math

class PopUpHelpDialog(urwid.Widget):
    _sizing = frozenset(['box'])

    def __init__(self):
        self.text = urwid.AttrMap(urwid.Filler(urwid.Text("blah blah")), "header")

    def render(self, size, focus=False):
        return self.text.render(size, focus)
        
    def keypress(self, size, key):
        return key

class HelpEntry(urwid.PopUpLauncher):
    _sizing = frozenset(['flow'])
    _selectable = False

    def __init__(self):
        self.__super.__init__(urwid.Text(""))
        self.open_flag = False

    def create_pop_up(self):
        pop_up = PopUpHelpDialog()
        self.open_flag = True
        return pop_up

    def get_pop_up_parameters(self):
        (screen_cols, screen_rows) = screen.get_cols_rows()
        width = screen_cols / 2
        height = screen_rows / 2
        return {'left':screen_cols / 4, 'top':screen_rows / 4, 'overlay_width':width, 'overlay_height':height}

class OutputEntry(urwid.Widget):
    _sizing = frozenset(['flow'])
    _selectable = False

    def __init__(self, row_offset):
        self.row_offset = row_offset
        self.canvas = urwid.TextCanvas("")
        self.init_flag = False

    def rows(self, size, focus):
        (display_cols,) = size
        (screen_cols, screen_rows) = screen.get_cols_rows()
        display_rows = screen_rows - self.row_offset

        if not self.init_flag:
            return display_rows
        else:
            return self.canvas.rows()

    def render(self, size, focus):
        (display_cols,) = size
        (screen_cols, screen_rows) = screen.get_cols_rows()
        display_rows = screen_rows - self.row_offset
            
        self.init_flag = True

        # check if display size has changed
        if self.canvas.cols() != display_cols:
            text = urwid.Filler(urwid.Text(""), valign='top', min_height=display_rows)
            new_canvas = text.render((display_cols, display_rows), False)
            self.canvas = urwid.CanvasCombine([(new_canvas, 'above', False)])
            
        return self.canvas

class TaskEntry(urwid.Widget):
    _sizing = frozenset(['flow'])
    _selectable = True

    def __init__(self):
        self.row_offset = 5
        self.focused = False
        self.name = urwid.AttrMap(urwid.Text(" " + "name", wrap='clip'), "body")
        self.status = urwid.AttrMap(urwid.Text(" " + "status", wrap='clip'), "status_nominal")
        self.load = urwid.AttrMap(urwid.Text(" " + "load", wrap='clip'), "body")
        self.memory = urwid.AttrMap(urwid.Text(" " + "memory", wrap='clip'), "body")
        self.message = urwid.AttrMap(urwid.Text(" " + "message", wrap="clip"), "body")
        self.stdout_full = ""
        self.stdout_buffer = ""
        self.stdout_canvas_full = urwid.TextCanvas("")
        self.stdout_canvas_fit = urwid.TextCanvas("")
        self.stdout_tip = 0
        self.stdout_tail = 0
        self.auto_scroll_flag = True
        self.count = 0
        self.header_flag = False
        self.hidden_rows = 0

        self.divider = (1, urwid.AttrMap(urwid.Text("|"), "body"))
        self.column_widths = {"name" : 20,
                              "status" : 20,
                              "load" : 6,
                              "memory" : 8}

    def SetDividerPalette(self, palette):
        self.divider[1].set_attr_map({None : palette})
        
    def rows(self, size, focus=False):
        return 1

    def SetRowOffset(self, row_offset):
        self.row_offset = row_offset

    def SetName(self, name):
        self.name.original_widget.set_text(" " + name)

    def SetNamePalette(self, palette):
        self.name.set_attr_map({None : palette})

    def SetStatus(self, status):
        self.status.original_widget.set_text(" " + status)

    def SetStatusPalette(self, palette):
        self.status.set_attr_map({None : palette})
        
    def SetLoad(self, load):
        self.load.original_widget.set_text(" " + format(load, "02.1f").zfill(4))

    def SetLoadPalette(self, palette):
        self.load.set_attr_map({None : palette})
        
    def SetLoadString(self, load):
        self.load.original_widget.set_text(" " + load)
        
    def SetMemory(self, memory):
        self.memory.original_widget.set_text(" " + format(memory, "02.1f").zfill(4))

    def SetMemoryPalette(self, palette):
        self.memory.set_attr_map({None : palette})
        
    def SetMemoryString(self, memory):
        self.memory.original_widget.set_text(" " + memory)
        
    def SetMessage(self, message):
        self.message.original_widget.set_text(" " + message)

    def SetMessagePalette(self, palette):
        self.message.set_attr_map({None : palette})
        
    def AddStdOut(self, stdout):
        self.stdout_buffer = self.stdout_buffer + stdout
        
    def SetFocus(self):
        self.focused = True
        self.name.set_attr_map({None : "selected"})
        self.load.set_attr_map({None : "selected"})
        self.memory.set_attr_map({None : "selected"})
        self.message.set_attr_map({None : "selected"})

    def UnsetFocus(self):
        self.focused = False
        self.name.set_attr_map({None : "body"})
        self.load.set_attr_map({None : "body"})
        self.memory.set_attr_map({None : "body"})
        self.message.set_attr_map({None : "body"})
        
    def BuildTaskEntry(self):
        column_list = []

        # name field
        column_list.append((self.column_widths["name"], self.name))
        column_list.append(self.divider)

        # status field
        column_list.append((self.column_widths["status"], self.status))
        column_list.append(self.divider)

        # load field
        column_list.append((self.column_widths["load"], self.load))
        column_list.append(self.divider)
        
        # memory field
        column_list.append((self.column_widths["memory"], self.memory))
        column_list.append(self.divider)

        # message field
        column_list.append(self.message)

        return urwid.Columns(column_list)

    def UpdateText(self, size, focus):
        (display_cols,) = size
        (screen_cols, screen_rows) = screen.get_cols_rows()
        display_rows = screen_rows - self.row_offset

        # update text and canvas
        if self.stdout_buffer:
            self.stdout_full = self.stdout_full + self.stdout_buffer
            self.stdout_buffer = ""
            new_text_full = urwid.Text(self.stdout_full)
            new_canvas_full = new_text_full.render(size, False)
            self.stdout_canvas_full = urwid.CanvasCombine([(new_canvas_full, 'above', False)])

        # place tip and tail based given auto scroll state
        text_rows = self.stdout_canvas_full.rows()
        if self.auto_scroll_flag:
            self.stdout_tip = text_rows
            if self.stdout_tip >= display_rows:
                self.stdout_tail = self.stdout_tip - display_rows
            else:
                self.stdout_tail = 0
        else:
            if self.stdout_tail < 0:
                self.stdout_tail = 0
            if self.stdout_tail > text_rows - display_rows:
                self.stdout_tail = text_rows - display_rows
            self.stdout_tip = self.stdout_tail + display_rows

        # copy full canvas and trim it to fit display
        self.stdout_canvas_fit = urwid.CanvasCombine([(self.stdout_canvas_full, None, False)])
        self.stdout_canvas_fit.trim(self.stdout_tail, self.stdout_tip - self.stdout_tail)

    def render(self, size, focus):
        (maxcol,) = size

        if not self.header_flag:
            if focus:
                self.SetFocus()
            else:
                self.UnsetFocus()
            self.UpdateText(size, focus)
            
        return self.BuildTaskEntry().render(size, focus)

    def keypress(self, size, key):
        (maxcol,) = size

        if key == 'enter':
            self.stdout_buffer = self.stdout_buffer + str(self.count)
            self.count = self.count + 1
            self.UpdateText(size, True)
        elif key == 'page up':
            self.auto_scroll_flag = False
            self.stdout_tail = self.stdout_tail - 1
            self.UpdateText(size, True)
        elif key == 'page down':
            if not self.auto_scroll_flag:
                self.stdout_tail = self.stdout_tail + 1
            self.UpdateText(size, True)
        elif key == ' ':
            self.auto_scroll_flag = True
            self.UpdateText(size, True)
        else:
            return key

    def selectable(self):
        return self._selectable
    
    def mouse_event(self, size, event, button, col, row, focus):
        if focus:
            if event == 'mouse press':
                return True
        
def UnhandledInputCallback(key):
    if key in ('q', 'Q'):
        raise urwid.ExitMainLoop()
    elif key in ('h', 'H'):
        if not help_entry.open_flag:
            help_entry.open_pop_up()
        else:
            help_entry.close_pop_up()
            help_entry.open_flag = False

def RefreshScreen(_loop, _data):
    if task1.focused:
        output_entry.canvas = task1.stdout_canvas_fit
    if task2.focused:
        output_entry.canvas = task2.stdout_canvas_fit

    output_entry._invalidate()

    UpdateTaskInfo()

    _loop.draw_screen()
    _loop.set_alarm_in(0.1, RefreshScreen)

def UpdateTaskInfo():
    for t in tasks:
        timestamp = time.time()

        val = 50.0 * math.sin(timestamp / 5.0) + 50.0
        int_val = int(val / 20.0) + 1
        t.SetLoad(val)
        t.SetMemory(val)
        output_string = ''.join(["blah" for s in xrange(int_val)])
        t.SetMessage(output_string)

        if int_val == 1:
            t.SetStatusPalette("status_nominal")
            t.SetStatus("Nominal")

        if int_val == 3:
            t.SetStatusPalette("status_warning")
            t.SetStatus("Warning")

        if int_val == 5:
            t.SetStatusPalette("status_error")
            t.SetStatus("Error")
            
        if (int(timestamp) % 2) == 0:
            timestamp_text = timestamp
            t.AddStdOut(str(val) + ": " + lorem.paragraph() + "\n")
    
if __name__ == '__main__':
    palette = [('body','light gray','default'),
               ('header','black','dark cyan'),
               ('selected','white','dark blue'),
               ('group','black','dark blue'),
               ('status_nominal', 'black', 'dark green'),
               ('status_warning', 'black', 'yellow'),
               ('status_error', 'black', 'dark red')]

    header = TaskEntry()
    header.header_flag = True
    header.SetName("Name")
    header.SetStatus("Status")
    header.SetLoadString("Load")
    header.SetMemoryString("Memory")
    header.SetMessage("Message")

    header.SetNamePalette("header")
    header.SetStatusPalette("header")
    header.SetLoadPalette("header")
    header.SetMemoryPalette("header")
    header.SetMessagePalette("header")
    header.SetDividerPalette("header")
    header._selectable = False
    header.stdout_buffer = "header\n\n"

    task1 = TaskEntry()
    task1.SetName("task1")
    task1.stdout_buffer = "text 1\n\n"

    task2 = TaskEntry()
    task2.SetName("task2")
    task2.stdout_buffer = "text 2\n\n"

    tasks = []
    tasks.append(task1)
    tasks.append(task2)

    entry_list = []
    entry_list.append(header)
    entry_list.append(task1)
    entry_list.append(task2)
    entry_list.append(urwid.Divider("_"))
    entry_list.append(urwid.Divider(" "))

    row_offset = len(entry_list) + 1  # add row for footer
    header.SetRowOffset(row_offset)
    task1.SetRowOffset(row_offset)
    task2.SetRowOffset(row_offset)
    
    output_entry = OutputEntry(row_offset)
    entry_list.append(output_entry)

    help_entry = HelpEntry()
    entry_list.append(help_entry)

    list_walker_content = [urwid.Pile(entry_list)]
    list_walker = urwid.SimpleFocusListWalker(list_walker_content)
    listbox = urwid.ListBox(list_walker)

    footer = urwid.AttrMap(urwid.Text(" " + "q:quit | h:help | page-up:scroll up | page-down:scroll down | space:auto scroll"), 'header')
    
    frame = urwid.Frame(body=listbox, footer=footer)
    screen = urwid.raw_display.Screen()
    urwid_main = urwid.MainLoop(frame,
                                palette,
                                screen,
                                handle_mouse=True,
                                unhandled_input=UnhandledInputCallback, pop_ups=True)
    urwid_main.set_alarm_in(0.1, RefreshScreen)
    urwid_main.run()

