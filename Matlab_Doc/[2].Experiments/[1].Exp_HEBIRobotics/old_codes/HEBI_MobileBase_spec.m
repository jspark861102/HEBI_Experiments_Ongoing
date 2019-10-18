clear all
close all
clc

m = 10;
mu = 0.5;
g = 9.81;

r = 0.10;

v = 0.9;
t = 0.5;
a = v/t;

w = v/r*60/2/pi

T = (mu*m*g*r + m*r*r*a)/2