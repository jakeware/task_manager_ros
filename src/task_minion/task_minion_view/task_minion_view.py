# Copyright 2018 Massachusetts Institute of Technology

from Tkinter import *
from ScrolledText import ScrolledText

class ProcessEntry:
    def __init__(self, parent, pid):
        self.pid = pid
        self.master_frame = Frame(parent)
        self.master_frame.pack(fill=X)
        self.name_text = Text(self.master_frame, state='disabled', height=1, width=10, highlightthickness=0, borderwidth=1, wrap='none')
        self.name_text.pack(side=LEFT)
        self.status_text = Text(self.master_frame, state='disabled', height=1, width=10, highlightthickness=0, borderwidth=1, wrap='none')
        self.status_text.pack(side=LEFT)
        self.load_text = Text(self.master_frame, state='disabled', height=1, width=10, highlightthickness=0, borderwidth=1, wrap='none')
        self.load_text.pack(side=LEFT)
        self.memory_text = Text(self.master_frame, state='disabled', height=1, width=10, highlightthickness=0, borderwidth=1, wrap='none')
        self.memory_text.pack(side=LEFT)
        self.message_text = Text(self.master_frame, state='disabled', height=1, highlightthickness=0, borderwidth=1, wrap='none')
        self.message_text.pack(side=LEFT,fill=X, expand=1)

        self.process_background_color_inactive_odd = 'white'
        self.process_background_color_inactive_even = 'light sky blue'
        self.process_background_color_active = 'blue'

    def SetActive(self):
        self.name_text['bg'] = self.process_background_color_active
        self.status_text['bg'] = self.process_background_color_active
        self.load_text['bg'] = self.process_background_color_active
        self.memory_text['bg'] = self.process_background_color_active
        self.message_text['bg'] = self.process_background_color_active

    def SetInactive(self):
        color = self.process_background_color_inactive_odd
        if self.pid % 2 == 0:
            color = self.process_background_color_inactive_even

        self.name_text['bg'] = color
        self.status_text['bg'] = color
        self.load_text['bg'] = color
        self.memory_text['bg'] = color
        self.message_text['bg'] = color

class TaskMinionView:
    def __init__(self, root):
        self.root = root
        self.root.title("Task Master")

        self.process_entries = []
        self.process_frame = Frame(self.root)
        self.process_frame.pack(fill=X)
        self.output_text = ScrolledText(self.root)
        self.output_text.pack(fill=BOTH, expand=1)

    def AddProcessEntry(self, pid):
        self.process_entries.append(ProcessEntry(self.process_frame, pid))

    def SetProcessActive(self, pid):
        self.process_entries[pid].SetActive()

    def SetProcessInactive(self, pid):
        self.process_entries[pid].SetInactive()