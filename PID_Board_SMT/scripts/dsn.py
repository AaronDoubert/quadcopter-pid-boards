import pcbnew, sys
b = pcbnew.LoadBoard(sys.argv[1])
print("netclasses", list(b.GetDesignSettings().m_NetSettings.GetNetclasses().keys()) if hasattr(b.GetDesignSettings().m_NetSettings, "GetNetclasses") else "?")
print("export", pcbnew.ExportSpecctraDSN(b, sys.argv[2]))
