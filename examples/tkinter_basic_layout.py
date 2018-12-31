from Tkinter import *
from ScrolledText import ScrolledText

# root frame
root = Tk()
root.rowconfigure(1, weight=1)
root.columnconfigure(0, weight=1)

# process data frame
process_frame = Frame(root)
process_frame.columnconfigure(9, weight=1)
process_frame.grid(row=0, column=0, sticky=N+E+W)

# add process widgets
for row in range(2):
    for col in range(10):
        if col < 9:
        	process_text = Text(process_frame, state='disabled', height=1, width=10, highlightthickness=0, borderwidth=1, wrap='none')
        	process_text.grid(row=row, column=col)
        else:
        	process_text = Text(process_frame, state='disabled', height=1, highlightthickness=0, borderwidth=1, wrap='none')
        	process_text.grid(row=row, column=col, sticky=E+W)

# process text frame
text_frame = Frame(root)
text_frame.rowconfigure(0, weight=1)
text_frame.columnconfigure(0, weight=1)
text_frame.grid(row=1, column=0, sticky=N+E+S+W)

# process text widget
output_text = ScrolledText(text_frame)
output_text.grid(row=0, column=0, sticky=N+E+S+W)

root.mainloop()