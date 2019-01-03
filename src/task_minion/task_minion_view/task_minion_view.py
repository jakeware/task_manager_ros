# Copyright 2018 Massachusetts Institute of Technology

from Tkinter import *
from ScrolledText import ScrolledText

class HeaderEntry:
    def __init__(self, parent_frame):
        self.master_frame = Frame(parent_frame)
        self.master_frame.grid(row=0, column=0, sticky=NSEW)

        self.name_text = Text(self.master_frame, height=1, width=10, highlightthickness=0, borderwidth=1, wrap='none')
        self.name_text.pack(side=LEFT)
        self.name_text.tag_configure("center", justify='center')
        self.name_text.insert('1.0', 'Name')
        self.name_text.tag_add("center", "1.0", END)
        self.name_text.config(state='disabled')
        
        self.status_text = Text(self.master_frame, height=1, width=10, highlightthickness=0, borderwidth=1, wrap='none')
        self.status_text.pack(side=LEFT)
        self.status_text.tag_configure("center", justify='center')
        self.status_text.insert('1.0', 'Status')
        self.status_text.tag_add("center", "1.0", END)
        self.status_text.config(state='disabled')
        
        self.load_text = Text(self.master_frame, height=1, width=10, highlightthickness=0, borderwidth=1, wrap='none')
        self.load_text.pack(side=LEFT)
        self.load_text.tag_configure("center", justify='center')
        self.load_text.insert('1.0', 'Load')
        self.load_text.tag_add("center", "1.0", END)
        self.load_text.config(state='disabled')
        
        self.memory_text = Text(self.master_frame, height=1, width=10, highlightthickness=0, borderwidth=1, wrap='none')
        self.memory_text.pack(side=LEFT)
        self.memory_text.tag_configure("center", justify='center')
        self.memory_text.insert('1.0', 'Memory')
        self.memory_text.tag_add("center", "1.0", END)
        self.memory_text.config(state='disabled')
        
        self.message_text = Text(self.master_frame, height=1, highlightthickness=0, borderwidth=1, wrap='none')
        self.message_text.pack(side=LEFT,fill=X, expand=1)
        self.message_text.tag_configure("center", justify='center')
        self.message_text.insert('1.0', 'Message')
        self.message_text.tag_add("center", "1.0", END)
        self.message_text.config(state='disabled')

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

    def SetTaskLoad(self, task_load):
        self.load_text.config(state='normal')
        self.load_text.insert('1.0', str(task_load))
        self.load_text.config(state='disabled')

    def SetTaskMemory(self, task_memory):
        self.memory_text.config(state='normal')
        self.memory_text.insert('1.0', str(task_memory))
        self.memory_text.config(state='disabled')

class OutputEntry:
    def __init__(self, parent_frame, task_id):
        self.task_id = task_id

        self.master_frame = Frame(parent_frame)
        self.master_frame.grid(row=2, column=0, sticky=NSEW)
        self.master_frame.grid_rowconfigure(0, weight=1)
        self.master_frame.grid_columnconfigure(0, weight=1)
        self.output_text = ScrolledText(self.master_frame)
        self.output_text.grid(row=0, column=0, sticky=NSEW)
        self.output_text.insert('1.0', "Task ID:" + str(task_id))
        self.output_text.config(state='disabled')

    def Raise(self):
        print "Raise ID:" + str(self.task_id)
        self.master_frame.lift()

class TaskMinionView:
    def __init__(self, root):
        self.task_entries = {}  # dictionary (indexed by task_id) of TaskEntries
        self.output_entries = {}  # dictionary (indexed by task_id) of OutputEntries
        self.task_order = []  # list of task ids in order they are displayed

        # layout
        self.root = root
        self.root.title("Task Master")
        self.root.grid_rowconfigure(2, weight=1)
        self.root.grid_columnconfigure(0, weight=1)
        self.header = HeaderEntry(self.root)
        self.task_frame = Frame(self.root)
        self.task_frame.grid(row=1, column=0, sticky=NSEW)

    def AddTask(self, task_id, task_name, task_depth):
        self.task_entries[task_id] = TaskEntry(self.task_frame, task_id, task_name, task_depth)
        self.output_entries[task_id] = OutputEntry(self.root, task_id)
        self.task_order.append(task_id)

    def SetTaskActiveByIndex(self, task_index):
        task_id = TaskIndexToId(task_index)
        self.SetTaskActiveById(task_id)

    def SetTaskInactiveByIndex(self, task_index):
        task_id = TaskIndexToId(task_index)
        self.SetTaskInactiveById(task_id)

    def SetTaskActiveById(self, task_id):
        self.output_entries[task_id].Raise()
        self.task_entries[task_id].SetActive()

    def SetTaskInactiveById(self, task_id):
        self.task_entries[task_id].SetInactive()

    def GetTaskEntryCount(self):
        return len(self.task_entries)

    def TaskIndexToId(self, task_index):
        if task_index < len(self.task_order):
            return self.task_order[task_index]
        else:
            print "[TaskMinionModel::TaskIndexToId] Index out of bounds:" + str(task_index)

    def SetTaskLoad(self, task_id, task_load):
        if task_id in self.task_entries:
            task_entry = self.task_entries[task_id]
            task_entry.SetTaskLoad(task_load)
        else:
            print "[TaskMinionModel::SetTaskLoad] Missing id:" + str(task_id)

    def SetTaskMemory(self, task_id, task_memory):
        if task_id in self.task_entries:
            task_entry = self.task_entries[task_id]
            task_entry.SetTaskMemory(task_memory)
        else:
            print "[TaskMinionModel::SetTaskMemory] Missing id:" + str(task_id)

    # def SetTaskOutput(self, task_stdout):
    #     self.output_text.config(state='normal')
    #     self.output_text.delete('1.0', END)
    #     self.output_text.insert('1.0', task_stdout)
    #     self.output_text.config(state='disabled')