Imports System.IO
Imports VehicleSim

Module VS_Run

  Sub Main(ByVal cmdArgs() As String)
    Console.WriteLine("VB Program that runs the simulation in simfile.sim in the current directory.")
    Console.WriteLine("")

    Dim pathToSimfile As String = Path.Combine(Directory.GetCurrentDirectory(), "simfile.sim")
    If cmdArgs.Length = 1 Then
      pathToSimfile = cmdArgs(0)
    End If

    Dim vs As Simulation = New Simulation()
    Dim pathToVsDll = vs.get_dll_path(pathToSimfile)

    If pathToVsDll IsNot String.Empty And File.Exists(pathToVsDll) Then
      Dim vsDll As IntPtr = vs.LoadLibrary(pathToVsDll)
      If vsDll <> IntPtr.Zero Then
        If vs.get_api(vsDll) Then
          vs.run(pathToSimfile)
        End If
        vs.UnloadLibrary(vsDll)
      End If
    End If
  End Sub

End Module
