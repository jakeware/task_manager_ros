# Copyright 2018 Massachusetts Institute of Technology

from Tkinter import *
from ScrolledText import ScrolledText

class HeaderEntry(object):
    def __init__(self, parent_frame):
        self.master_frame = Frame(parent_frame)
        self.master_frame.grid(row=0, column=0, sticky=NSEW)
        self.bg_color = 'navy'
        self.fg_color = 'white'

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
        
        self.master_frame = Frame(parent_frame)
        self.master_frame.pack(fill=X)
        self.fg_color = 'white'
        self.bg_color_inactive = 'black'
        self.bg_color_active = 'gray'

        depth_indent = task_depth * '  '
        self.name_text = Text(self.master_frame, height=1, width=10, highlightthickness=0, borderwidth=0, wrap='none', bg='black', fg=self.fg_color)
        self.name_text.pack(side=LEFT)
        self.name_text.insert('1.0', depth_indent + task_name)
        self.name_text.config(state='disabled')
        
        self.status_text = Text(self.master_frame, state='disabled', height=1, width=10, highlightthickness=0, borderwidth=0, wrap='none', bg='black', fg=self.fg_color)
        self.status_text.pack(side=LEFT)
        
        self.load_text = Text(self.master_frame, state='disabled', height=1, width=10, highlightthickness=0, borderwidth=0, wrap='none', bg='black', fg=self.fg_color)
        self.load_text.pack(side=LEFT)
        
        self.memory_text = Text(self.master_frame, state='disabled', height=1, width=10, highlightthickness=0, borderwidth=0, wrap='none', bg='black', fg=self.fg_color)
        self.memory_text.pack(side=LEFT)
        
        self.message_text = Text(self.master_frame, state='disabled', height=1, highlightthickness=0, borderwidth=0, wrap='none', bg='black', fg=self.fg_color)
        self.message_text.pack(side=LEFT,fill=X, expand=1)

        self.SetInactive()        

    def SetActive(self):
        self.name_text['bg'] = self.bg_color_active
        status_text = self.status_text.get("1.0",END)
        if status_text.isspace():
            self.status_text['bg'] = self.bg_color_active
            self.status_text['fg'] = self.fg_color
        self.load_text['bg'] = self.bg_color_active
        self.memory_text['bg'] = self.bg_color_active
        self.message_text['bg'] = self.bg_color_active

    def SetInactive(self):
        self.name_text['bg'] = self.bg_color_inactive
        status_text = self.status_text.get("1.0",END)
        if status_text.isspace():
            self.status_text['bg'] = self.bg_color_inactive
            self.status_text['fg'] = self.fg_color
        self.load_text['bg'] = self.bg_color_inactive
        self.memory_text['bg'] = self.bg_color_inactive
        self.message_text['bg'] = self.bg_color_inactive

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

        self.master_frame = Frame(parent_frame)
        self.master_frame.grid(row=2, column=0, sticky=NSEW)
        self.master_frame.grid_rowconfigure(0, weight=1)
        self.master_frame.grid_columnconfigure(0, weight=1)
        self.output_text = ScrolledText(self.master_frame, bg=self.bg_color, fg=self.fg_color)
        self.output_text.grid(row=0, column=0, sticky=NSEW)
        self.output_text.insert('1.0', "Task ID:" + str(task_id))
        self.output_text.config(state='disabled')

    def SetActive(self):
        print "Raise ID:" + str(self.task_id)
        self.master_frame.lift()

    def SetTaskOutput(self, task_output):
        self.output_text.config(state='normal')
        self.output_text.delete('1.0', END)
        self.output_text.insert('1.0', task_output)
        self.output_text.see(END)
        self.output_text.config(state='disabled')

    def SetTaskOutputDelta(self, task_output):
        self.output_text.config(state='normal')
        self.output_text.insert(END, task_output)
        self.output_text.see(END)
        self.output_text.config(state='disabled')

class TaskMinionView(object):
    def __init__(self, root):
        self.task_entries = {}  # dictionary (indexed by task_id) of TaskEntries
        self.output_entries = {}  # dictionary (indexed by task_id) of OutputEntries
        self.task_order = []  # list of task ids in order they are displayed

        # layout
        self.root = root
        self.root.title("Task Minion")
        self.root.grid_rowconfigure(2, weight=1)
        self.root.grid_columnconfigure(0, weight=1)
        self.header = HeaderEntry(self.root)
        self.task_frame = Frame(self.root)
        self.task_frame.grid(row=1, column=0, sticky=NSEW)

    def AddTask(self, task_id, task_name, task_depth):
        self.task_entries[task_id] = TaskEntry(self.task_frame, task_id, task_name, task_depth)
        self.output_entries[task_id] = OutputEntry(self.root, task_id)
        self.task_order.append(task_id)

    def SetTaskAndOutputActiveById(self, task_id):
        if task_id in self.output_entries:
            self.output_entries[task_id].SetActive()
            self.SetTaskActiveById(task_id)
        else:
            print "[TaskMinionModel::SetTaskAndOutputActiveById] Missing id:" + str(task_id)

    def SetTaskActiveById(self, task_id):
        if task_id in self.task_entries:
            self.task_entries[task_id].SetActive()
        else:
            print "[TaskMinionModel::SetTaskActiveById] Missing id:" + str(task_id)

    def SetTaskInactiveById(self, task_id):
        if task_id in self.task_entries:
            self.task_entries[task_id].SetInactive()
        else:
            print "[TaskMinionModel::SetTaskInactiveById] Missing id:" + str(task_id)

    def GetTaskEntryCount(self):
        return len(self.task_entries)

    def TaskIndexToId(self, task_index):
        if task_index < len(self.task_order):
            return self.task_order[task_index]
        else:
            print "[TaskMinionModel::TaskIndexToId] Index out of bounds:" + str(task_index)

    def SetTaskStatusById(self, task_id, task_status):
        if task_id in self.task_entries:
            task_entry = self.task_entries[task_id]
            task_entry.SetTaskStatus(task_status)
        else:
            print "[TaskMinionModel::SetTaskLoadById] Missing id:" + str(task_id)

    def SetTaskLoadById(self, task_id, task_load):
        if task_id in self.task_entries:
            task_entry = self.task_entries[task_id]
            task_entry.SetTaskLoad(task_load)
        else:
            print "[TaskMinionModel::SetTaskLoadById] Missing id:" + str(task_id)

    def SetTaskMemoryById(self, task_id, task_memory):
        if task_id in self.task_entries:
            task_entry = self.task_entries[task_id]
            task_entry.SetTaskMemory(task_memory)
        else:
            print "[TaskMinionModel::SetTaskMemoryById] Missing id:" + str(task_id)

    def SetTaskOutputById(self, task_id, task_output):
        if task_id in self.output_entries:
            output_entry = self.output_entries[task_id]
            output_entry.SetTaskOutput(task_output)
        else:
            print "[TaskMinionModel::SetTaskOutputById] Missing id:" + str(task_id)

    def SetTaskOutputDeltaById(self, task_id, task_output_delta):
        if task_id in self.output_entries:
            output_entry = self.output_entries[task_id]
            output_entry.SetTaskOutputDelta(task_output_delta)
        else:
            print "[TaskMinionModel::SetTaskOutputById] Missing id:" + str(task_id)