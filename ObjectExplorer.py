
import tkinter as tk
import tkinter.ttk as ttk
import inspect


class ObjectExplorer:
    def __init__(self, targetObject = None):
        self.rootObject = targetObject
        self.objectMap = {}
        self.selectedObject = None

        self.master = tk.Tk()
        self.master.title("Object Explorer")
        self.master.protocol("WM_DELETE_WINDOW", self.handleCloseButton)

        self.tree = ttk.Treeview(self.master, height=30, selectmode='browse', columns=["Object"])
        vsb = ttk.Scrollbar(self.master, orient="vertical", command=self.tree.yview)
        hsb = ttk.Scrollbar(self.master, orient="horizontal", command=self.tree.xview)
        self.tree.configure(yscrollcommand=vsb.set, xscrollcommand=hsb.set)
        self.tree.grid(column=0, row=0, sticky='nsew', in_=self.master)
        vsb.grid(column=1, row=0, sticky='ns', in_=self.master)
        hsb.grid(column=0, row=1, sticky='ew', in_=self.master)
        self.tree.column("Object", width=500)
        self.tree.bind("<Button-1>", self.handleSelect)
        self.tree.bind("<Double-1>", self.handleExpand)

        self.textVariable = tk.StringVar()
        self.textDisplay = tk.Label(self.master, height=5, textvariable=self.textVariable, bg="#fff",
                                    anchor=tk.SW, relief=tk.SUNKEN, bd=2, justify=tk.LEFT)
        self.textDisplay.grid(row=1, column=0, sticky='nsew')

        self.textInput = tk.Entry(self.master)
        self.textInput.grid(row=2, column=0, sticky='nsew')
        self.textInput.bind("<Key-Return>", self.evaluateText)

        top=self.master.winfo_toplevel()
        top.rowconfigure(0, weight=1)
        top.columnconfigure(0, weight=1)

        self.master.rowconfigure(0, weight=1)
        self.master.columnconfigure(0, weight=1)
        self.master.grid()
        self.buildTree(self.rootObject)

    def buildTree(self, target, parent=''):
        lastId = None
        if isinstance(target, list):
            for index, data in enumerate(target):
                iid = self.tree.insert(parent, 'end', text=index, open=True, values=(data,))
                self.objectMap[iid] = data
                lastId = iid
        elif isinstance(target, dict):
            for key in target:
                iid = self.tree.insert(parent, 'end', text=key, open=True, values=(target[key],))
                self.objectMap[iid] = target[key]
                lastId = iid
        else:
            for name, data in inspect.getmembers(target):
                if name.startswith("__") and name != "__class__":
                    continue
                if inspect.ismethod(data):
                    continue
                iid = self.tree.insert(parent, 'end', text=name, open=True, values=(data,))
                self.objectMap[iid] = data
                lastId = iid
        return lastId

    def evaluateText(self, event):
        text = self.textInput.get()
        self.textInput.delete(0, tk.END)
        try:
            result = eval(text, globals(), locals())
        except Exception as ex:
            result = ex
        self.textVariable.set(self.textVariable.get() + "\n" + str(result))

    def handleSelect(self, event):
        idPair = self.tree.selection()
        if idPair != "":
            id = idPair[0]
            self.selectedObject = self.objectMap[id]

    def handleExpand(self, event):
        id = self.tree.selection()[0]
        item = self.objectMap[id]
        # don't expand if this list item is already a parent
        for iid in self.objectMap:
            if self.tree.parent(iid) == id:
                return
        lastId = self.buildTree(item, id)
        self.master.after(10, self.selectId, lastId)

    def selectId(self, id):
        self.tree.see(id)

    def handleCloseButton(self):
        self.quit()

    def quit(self):
        self.master.destroy()
        self.master.quit()

    def open(self):
        tk.mainloop()

def explore(someObject):
    ObjectExplorer(someObject).open()
