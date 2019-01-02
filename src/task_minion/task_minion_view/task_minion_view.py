# Copyright 2018 Massachusetts Institute of Technology

from Tkinter import *
from ScrolledText import ScrolledText

class TaskEntry:
    def __init__(self, parent_frame, task_id, task_name, task_depth):
        self.task_id = task_id
        
        self.master_frame = Frame(parent_frame)
        self.master_frame.pack(fill=X)

        depth_indent = task_depth * '  '
        self.name_text = Text(self.master_frame, height=1, width=10, highlightthickness=0, borderwidth=1, wrap='none')
        self.name_text.pack(side=LEFT)
        self.name_text.insert('1.0', depth_indent + task_name)
        self.name_text.config(state='disabled')
        
        self.status_text = Text(self.master_frame, state='disabled', height=1, width=10, highlightthickness=0, borderwidth=1, wrap='none')
        self.status_text.pack(side=LEFT)
        
        self.load_text = Text(self.master_frame, state='disabled', height=1, width=10, highlightthickness=0, borderwidth=1, wrap='none')
        self.load_text.pack(side=LEFT)
        
        self.memory_text = Text(self.master_frame, state='disabled', height=1, width=10, highlightthickness=0, borderwidth=1, wrap='none')
        self.memory_text.pack(side=LEFT)
        
        self.message_text = Text(self.master_frame, state='disabled', height=1, highlightthickness=0, borderwidth=1, wrap='none')
        self.message_text.pack(side=LEFT,fill=X, expand=1)

        self.task_background_color_inactive = 'white'
        self.task_background_color_active = 'blue'

        self.SetInactive()

    def SetActive(self):
        color = self.task_background_color_active
        self.name_text['bg'] = color
        self.status_text['bg'] = color
        self.load_text['bg'] = color
        self.memory_text['bg'] = color
        self.message_text['bg'] = color

    def SetInactive(self):
        color = self.task_background_color_inactive
        self.name_text['bg'] = color
        self.status_text['bg'] = color
        self.load_text['bg'] = color
        self.memory_text['bg'] = color
        self.message_text['bg'] = color

class TaskMinionView:
    def __init__(self, root):
        self.root = root
        self.root.title("Task Master")
        self.task_entries = []  # ordered list of task entries
        self.task_frame = Frame(self.root)
        self.task_frame.pack(fill=X)
        self.output_text = ScrolledText(self.root)
        self.output_text.pack(fill=BOTH, expand=1)

    def SetTaskEntry(self, task_id, task_name, task_depth):
        self.task_entries.append(TaskEntry(self.task_frame, task_id, task_name, task_depth))

    def SetTaskActive(self, task_id):
        self.task_entries[task_id].SetActive()

    def SetTaskInactive(self, task_id):
        self.task_entries[task_id].SetInactive()

    def GetTaskEntryCount(self):
        return len(self.task_entries)