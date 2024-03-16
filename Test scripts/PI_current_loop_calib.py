import numpy    

R =.5; #% Resistancein Ohms
L = 1e-3; #% Inductancein Henries 
Ts =.00005; #% Sampleperiod
fs = 20000; # Sample frequency
#wc = numpy.pi/10; #%Crossover frequency,in Radiansper sample
current_loop_bw = 1000 #wc * fs/ (numpy.pi*2)
wc = current_loop_bw / (fs / (numpy.pi*2))
print("wc",wc)
print("current_loop_bw",current_loop_bw)

ki =  1 - numpy.exp(-R*Ts/L)
print("Ki",ki)
k = R * ((wc) / (1-numpy.exp(-R*Ts/L)))
print("K",k)


# Example of my cheap high pole pair motor 
R =7.61; #% Resistancein Ohms
L = 0.00501; #% Inductancein Henries 
Ts = 0.00014285; #% Sampleperiod
fs = 7000; # Sample frequency
#wc = numpy.pi/10; #%Crossover frequency,in Radiansper sample
current_loop_bw = 1000
#current_loop_bw = wc * fs/ (numpy.pi*2)
wc = current_loop_bw / (fs / (numpy.pi*2))
print("wc",wc)
print("current_loop_bw",current_loop_bw)

ki =  1 - numpy.exp(-R*Ts/L)
print("Ki",ki)
k = R * ((wc) / (1-numpy.exp(-R*Ts/L)))
print("K",k)