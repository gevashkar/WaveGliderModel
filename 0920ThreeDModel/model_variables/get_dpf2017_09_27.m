function dpf = get_dpf2017_09_27(in1,in2)
%GET_DPF2017_09_27
%    DPF = GET_DPF2017_09_27(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 5.10.
%    27-Sep-2017 17:07:37

dxf = in2(1,:);
dyf = in2(2,:);
dzf = in2(3,:);
dpf = [dxf;dyf;dzf];
