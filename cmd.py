print "Command is:",command
op = subprocess.Popen(command, shell=True, stderr=subprocess.PIPE, stdout=subprocess.PIPE)
if op:
    output=str(op.stdout.read())
    print "Output:",output
    conn.sendall(output)

else:
    error=str(op.stderr.read())
    print "Error:",error
    conn.sendall(error)