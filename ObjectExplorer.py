
import tkinter as tk
import tkinter.ttk as ttk
import inspect


class ObjectExplorer:
    def __init__(self, targetObject = None):
        self.rootObject = targetObject
        self.objectMap = {}

        self.master = tk.Tk()
        self.master.title("Object Explorer")
        self.master.protocol("WM_DELETE_WINDOW", self.handleCloseButton)

        self.tree = ttk.Treeview(height=30, selectmode='browse', columns=["Object"])
        vsb = ttk.Scrollbar(self.master, orient="vertical", command=self.tree.yview)
        hsb = ttk.Scrollbar(self.master, orient="horizontal", command=self.tree.xview)
        self.tree.configure(yscrollcommand=vsb.set, xscrollcommand=hsb.set)
        self.tree.grid(column=0, row=0, sticky='nsew', in_=self.master)
        vsb.grid(column=1, row=0, sticky='ns', in_=self.master)
        hsb.grid(column=0, row=1, sticky='ew', in_=self.master)
        top=self.master.winfo_toplevel()
        top.rowconfigure(0, weight=1)
        top.columnconfigure(0, weight=1)
        self.tree.column("Object", width=500)
        self.master.rowconfigure(0, weight=1)
        self.master.columnconfigure(0, weight=1)
        self.tree.bind("<Double-1>", self.handleSelect)
        self.master.grid()
        self.buildTree(self.rootObject)

    def buildTree(self, target, parent=''):
        lastId = None
        if type(target) is list:
            for index, data in enumerate(target):
                iid = self.tree.insert(parent, 'end', text=index, open=True, values=(data,))
                #self.tree.see(iid)
                self.objectMap[iid] = data
                lastId = iid
        else:
            for name, data in inspect.getmembers(target):
                if name.startswith("__") and name != "__class__":
                    continue
                if inspect.ismethod(data):
                    continue
                iid = self.tree.insert(parent, 'end', text=name, open=True, values=(data,))
                #self.tree.see(iid)
                self.objectMap[iid] = data
                lastId = iid
        return lastId

    def handleSelect(self, event):
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
