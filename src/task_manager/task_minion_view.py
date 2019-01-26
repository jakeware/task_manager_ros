# Copyright 2018 Massachusetts Institute of Technology

from Tkinter import *
from ScrolledText import ScrolledText

class HeaderEntry(object):
    def __init__(self, parent_frame):
        self.master_frame = Frame(parent_frame)
        self.master_frame.grid(row=0, column=0, sticky=NSEW)
        self.bg_color = 'slate gray'
        self.fg_color = 'black'

        self.name_text = Text(self.master_frame, height=1, width=10, highlightthickness=0, borderwidth=0, wrap='none', bg=self.bg_color, fg=self.fg_color)
        self.name_text.pack(side=LEFT)
        self.name_text.tag_configure("center", justify='center')
        self.name_text.insert('1.0', 'Name')
        self.name_text.tag_add("center", "1.0", END)
        self.name_text.config(state='disabled')
        
        self.status_text = Text(self.master_frame, height=1, width=10, highlightthickness=0, borderwidth=0, wrap='none', bg=self.bg_color, fg=self.fg_color)
        self.status_text.pack(side=LEFT)
        self.status_text.tag_configure("center", justify='center')
        self.status_text.insert('1.0', 'Status')
        self.status_text.tag_add("center", "1.0", END)
        self.status_text.config(state='disabled')
        
        self.load_text = Text(self.master_frame, height=1, width=10, highlightthickness=0, borderwidth=0, wrap='none', bg=self.bg_color, fg=self.fg_color)
        self.load_text.pack(side=LEFT)
        self.load_text.tag_configure("center", justify='center')
        self.load_text.insert('1.0', 'Load')
        self.load_text.tag_add("center", "1.0", END)
        self.load_text.config(state='disabled')
        
        self.memory_text = Text(self.master_frame, height=1, width=10, highlightthickness=0, borderwidth=0, wrap='none', bg=self.bg_color, fg=self.fg_color)
        self.memory_text.pack(side=LEFT)
        self.memory_text.tag_configure("center", justify='center')
        self.memory_text.insert('1.0', 'Memory')
        self.memory_text.tag_add("center", "1.0", END)
        self.memory_text.config(state='disabled')
        
        self.message_text = Text(self.master_frame, height=1, highlightthickness=0, borderwidth=0, wrap='none', bg=self.bg_color, fg=self.fg_color)
        self.message_text.pack(side=LEFT,fill=X, expand=1)
        self.message_text.tag_configure("center", justify='center')
        self.message_text.insert('1.0', 'Message')
        self.message_text.tag_add("center", "1.0", END)
        self.message_text.config(state='disabled')

class TaskEntry(object):
    def __init__(self, parent_frame, task_id, task_name, task_depth):
        self.task_id = task_id
        
        self.fg_color = 'steel blue'
        self.bg_color = 'black'
        self.fg_color_activated = 'white'
        self.fg_color_selected = 'black'
        self.bg_color_selected = 'steel blue'
        self.hl_color_focused = 'white'

        self.master_frame = Frame(parent_frame, highlightthickness=1, highlightbackground=self.bg_color)
        self.master_frame.pack(fill=X)

        depth_indent = task_depth * '  '
        self.name_text = Text(self.master_frame, height=1, width=10, highlightthickness=0, borderwidth=0, wrap='none', bg=self.bg_color, fg=self.fg_color)
        self.name_text.pack(side=LEFT)
        self.name_text.insert('1.0', depth_indent + task_name)
        self.name_text.config(state='disabled')
        
        self.status_text = Text(self.master_frame, state='disabled', height=1, width=10, highlightthickness=0, borderwidth=0, wrap='none', bg=self.bg_color, fg=self.fg_color)
        self.status_text.pack(side=LEFT)
        
        self.load_text = Text(self.master_frame, state='disabled', height=1, width=10, highlightthickness=0, borderwidth=0, wrap='none', bg=self.bg_color, fg=self.fg_color)
        self.load_text.pack(side=LEFT)
        
        self.memory_text = Text(self.master_frame, state='disabled', height=1, width=10, highlightthickness=0, borderwidth=0, wrap='none', bg=self.bg_color, fg=self.fg_color)
        self.memory_text.pack(side=LEFT)
        
        self.message_text = Text(self.master_frame, state='disabled', height=1, highlightthickness=0, borderwidth=0, wrap='none', bg=self.bg_color, fg=self.fg_color)
        self.message_text.pack(side=LEFT,fill=X, expand=1)

        self.focused = False
        self.activated = False
        self.selected = False

        self.Reset()        

    def Select(self):
        self.selected = True
        self.Refresh()

    def Deselect(self):
        self.selected = False
        self.Refresh()

    def Focus(self):
        self.focused = True
        self.Refresh()

    def Defocus(self):
        self.focused = False 
        self.Refresh()

    def Activate(self):
        self.activated = True
        self.Refresh()

    def Deactivate(self):
        self.activated = False
        self.Refresh()

    def Refresh(self):
        self.Reset()

        if self.selected:
            self.name_text['fg'] = self.fg_color_selected
            self.name_text['bg'] = self.bg_color_selected

        if self.activated:
            self.name_text['fg'] = self.fg_color_activated

        if self.focused:
            self.master_frame.config(highlightbackground=self.hl_color_focused)

    def Reset(self):
        self.name_text['fg'] = self.fg_color
        self.name_text['bg'] = self.bg_color
        status_text = self.status_text.get("1.0",END)
        if status_text.isspace():
            self.status_text['bg'] = self.bg_color
            self.status_text['fg'] = self.fg_color
        self.load_text['bg'] = self.bg_color
        self.memory_text['bg'] = self.bg_color
        self.message_text['bg'] = self.bg_color
        self.master_frame.config(highlightbackground=self.bg_color)

    def SetTaskStatus(self, task_status):
        self.status_text.config(state='normal')
        self.status_text.delete('1.0', END)
        self.status_text.insert('1.0', task_status)
        self.status_text.tag_configure("center", justify='center')
        self.status_text.tag_add("center", 1.0, END)
        if task_status == 'running':
            self.status_text['bg'] = 'green'
        if task_status == 'stopped':
            self.status_text['bg'] = 'red'
        self.status_text['fg'] = 'black'
        self.status_text.config(state='disabled')

    def SetTaskLoad(self, task_load):
        self.load_text.config(state='normal')
        self.load_text.delete('1.0', END)
        self.load_text.insert('1.0', str(round(task_load,2)))
        self.load_text.tag_configure("center", justify='center')
        self.load_text.tag_add("center", 1.0, END)
        self.load_text.config(state='disabled')

    def SetTaskMemory(self, task_memory):
        self.memory_text.config(state='normal')
        self.memory_text.delete('1.0', END)
        self.memory_text.insert('1.0', str(round(task_memory,2)))
        self.memory_text.tag_configure("center", justify='center')
        self.memory_text.tag_add("center", 1.0, END)
        self.memory_text.config(state='disabled')

class OutputEntry(object):
    def __init__(self, parent_frame, task_id):
        self.task_id = task_id
        self.bg_color = 'black'
        self.fg_color = 'white'
        self.border_color = 'slate gray'

        self.master_frame = Frame(parent_frame)
        self.master_frame.grid(row=2, column=0, sticky=NSEW)
        self.master_frame.grid_rowconfigure(0, weight=1)
        self.master_frame.grid_columnconfigure(0, weight=1)
        self.output_text = ScrolledText(self.master_frame, bg=self.bg_color, fg=self.fg_color, highlightbackground=self.border_color)
        self.output_text.grid(row=0, column=0, sticky=NSEW)
        self.output_text.insert('1.0', "Task ID:" + str(task_id))
        self.output_text.config(state='disabled')

    def Focus(self):
        # print "Raise ID:" + str(self.task_id)
        self.master_frame.lift()

    def SetTaskOutput(self, task_output):
        self.output_text.config(state='normal')
        self.output_text.delete('1.0', END)
        self.output_text.insert('1.0', task_output)
        self.output_text.see(END)
        self.output_text.config(state='disabled')

    def SetTaskOutputDelta(self, task_output):
        window_position_fractions = self.output_text.yview()
        bottom_position_fraction = window_position_fractions[1]
        autoscroll = False
        if bottom_position_fraction == 1.0:
            autoscroll = True

        self.output_text.config(state='normal')
        self.output_text.insert(END, task_output)
        if autoscroll:
            self.output_text.see(END)
        self.output_text.config(state='disabled')

class TaskMinionView(object):
    def __init__(self, root):
        self.task_entries = {}  # dictionary (indexed by task_id) of TaskEntries
        self.output_entries = {}  # dictionary (indexed by task_id) of OutputEntries
        self.task_order = []  # list of task ids in order they are displayed
        self.border_color = 'slate gray'

        # layout
        self.root = root
        self.root.title("Task Minion")
        self.root.grid_rowconfigure(2, weight=1)
        self.root.grid_columnconfigure(0, weight=1)
        self.header = HeaderEntry(self.root)
        self.task_frame = Frame(self.root, highlightthickness=1, highlightbackground=self.border_color)
        self.task_frame.grid(row=1, column=0, sticky=NSEW)

    def AddTask(self, task_id, task_name, task_depth):
        self.task_entries[task_id] = TaskEntry(self.task_frame, task_id, task_name, task_depth)
        self.output_entries[task_id] = OutputEntry(self.root, task_id)
        self.task_order.append(task_id)

    def FocusTaskAndOutputById(self, task_id):
        if task_id in self.output_entries:
            self.output_entries[task_id].Focus()
            self.FocusTaskById(task_id)
        else:
            print "[TaskMinionView::SetTaskAndOutputActiveById] Missing id:" + str(task_id)

    def ActivateTaskById(self, task_id):
        if task_id in self.task_entries:
            self.task_entries[task_id].Activate()
        else:
            print "[TaskMinionView::ActivateTaskById] Missing id:" + str(task_id)

    def DeactivateTaskById(self, task_id):
        if task_id in self.task_entries:
            self.task_entries[task_id].Deactivate()
        else:
            print "[TaskMinionView::DeactivateTaskById] Missing id:" + str(task_id)

    def FocusTaskById(self, task_id):
        if task_id in self.task_entries:
            self.task_entries[task_id].Focus()
        else:
            print "[TaskMinionView::FocusTaskById] Missing id:" + str(task_id)

    def DefocusTaskById(self, task_id):
        if task_id in self.task_entries:
            self.task_entries[task_id].Defocus()
        else:
            print "[TaskMinionView::DefocusTaskById] Missing id:" + str(task_id)

    def SelectTaskById(self, task_id):
        if task_id in self.task_entries:
            self.task_entries[task_id].Select()
        else:
            print "[TaskMinionView::SelectTaskById] Missing id:" + str(task_id)

    def DeselectTaskById(self, task_id):
        if task_id in self.task_entries:
            self.task_entries[task_id].Deselect()
        else:
            print "[TaskMinionView::DeselectTaskById] Missing id:" + str(task_id)

    def ResetTaskById(self, task_id):
        if task_id in self.task_entries:
            self.task_entries[task_id].Reset()
        else:
            print "[TaskMinionView::ResetTaskById] Missing id:" + str(task_id)

    def GetTaskEntryCount(self):
        return len(self.task_entries)

    def TaskIndexToId(self, task_index):
        if task_index < len(self.task_order):
            return self.task_order[task_index]
        else:
            print "[TaskMinionView::TaskIndexToId] Index out of bounds:" + str(task_index)
            return None

    def TaskIdToIndex(self, task_id):
        if task_id in self.task_order:
            return self.task_order.index(task_id)
        else:
            print "[TaskMinionView::TaskIdToIndex] Missing id:" + str(task_id)
            return None

    def SetTaskStatusById(self, task_id, task_status):
        if task_id in self.task_entries:
            task_entry = self.task_entries[task_id]
            task_entry.SetTaskStatus(task_status)
        else:
            print "[TaskMinionView::SetTaskLoadById] Missing id:" + str(task_id)

    def SetTaskLoadById(self, task_id, task_load):
        if task_id in self.task_entries:
            task_entry = self.task_entries[task_id]
            task_entry.SetTaskLoad(task_load)
        else:
            print "[TaskMinionView::SetTaskLoadById] Missing id:" + str(task_id)

    def SetTaskMemoryById(self, task_id, task_memory):
        if task_id in self.task_entries:
            task_entry = self.task_entries[task_id]
            task_entry.SetTaskMemory(task_memory)
        else:
            print "[TaskMinionView::SetTaskMemoryById] Missing id:" + str(task_id)

    def SetTaskOutputById(self, task_id, task_output):
        if task_id in self.output_entries:
            output_entry = self.output_entries[task_id]
            output_entry.SetTaskOutput(task_output)
        else:
            print "[TaskMinionView::SetTaskOutputById] Missing id:" + str(task_id)

    def SetTaskOutputDeltaById(self, task_id, task_output_delta):
        if task_id in self.output_entries:
            output_entry = self.output_entries[task_id]
            output_entry.SetTaskOutputDelta(task_output_delta)
        else:
            print "[TaskMinionView::SetTaskOutputById] Missing id:" + str(task_id)
