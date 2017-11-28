function dpg = get_dpg2017_06_10(in1,in2)
%GET_DPG2017_06_10
%    DPG = GET_DPG2017_06_10(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 5.10.
%    11-Jun-2017 05:03:44

dphif = in2(4,:);
dphig = in2(9,:);
dphit = in2(7,:);
dpsif = in2(6,:);
dthetaf = in2(5,:);
dthetag = in2(10,:);
dthetat = in2(8,:);
dxf = in2(1,:);
dyf = in2(2,:);
dzf = in2(3,:);
phif = in1(4,:);
phig = in1(9,:);
phit = in1(7,:);
psif = in1(6,:);
thetaf = in1(5,:);
thetag = in1(10,:);
thetat = in1(8,:);
t2 = thetaf-thetat;
t3 = pi.*(1.0./2.0);
t4 = t3+thetag-thetat;
t5 = cos(t4);
t6 = phif-phig;
t7 = cos(t6);
t8 = phif-phit;
t9 = cos(t2);
t10 = cos(t8);
t11 = sin(t2);
t12 = sin(t4);
t13 = sin(phif);
t14 = sin(psif);
t15 = cos(phif);
t16 = cos(psif);
t17 = -t3+thetaf;
t18 = sin(t6);
t19 = sin(t8);
t20 = cos(t17);
t21 = t13.*t14;
t22 = t15.*t16.*t20;
t23 = t21+t22;
t24 = sin(t17);
t25 = t14.*t15;
t42 = t13.*t16.*t20;
t26 = t25-t42;
t27 = t10.*t18;
t28 = t7.*t9.*t19;
t29 = t27+t28;
t30 = t18.*t19;
t32 = t7.*t9.*t10;
t31 = t30-t32;
t33 = t12.*t31.*(2.1e1./2.0e2);
t34 = t5.*t10.*t11.*(2.1e1./2.0e2);
t35 = t9.*t19.*4.0;
t36 = t12.*t29.*(2.1e1./2.0e2);
t45 = t5.*t11.*t19.*(2.1e1./2.0e2);
t37 = t35+t36-t45;
t38 = t11.*4.0;
t39 = t5.*t9.*(2.1e1./2.0e2);
t40 = t7.*t11.*t12.*(2.1e1./2.0e2);
t41 = t38+t39+t40;
t43 = t7.*t19;
t44 = t9.*t10.*t18;
t46 = t7.*t10;
t47 = t11.*t12.*t18.*t26.*(2.1e1./2.0e2);
t49 = t9.*t10.*4.0;
t48 = t33+t34-t49-2.3e1./2.0e2;
t50 = t9.*4.0;
t51 = t9.*t12.*(2.1e1./2.0e2);
t52 = t7.*t9.*t12.*(2.1e1./2.0e2);
t53 = t5.*t31.*(2.1e1./2.0e2);
t54 = t10.*t11.*4.0;
t55 = t5.*t9.*t10.*(2.1e1./2.0e2);
t56 = t7.*t10.*t11.*t12.*(2.1e1./2.0e2);
t57 = t5.*t29.*(2.1e1./2.0e2);
t58 = t11.*t19.*4.0;
t59 = t5.*t9.*t19.*(2.1e1./2.0e2);
t60 = t11.*t12.*t19.*(2.1e1./2.0e2);
t61 = t7.*t11.*t12.*t19.*(2.1e1./2.0e2);
t62 = t15.*t16;
t63 = t13.*t14.*t20;
t64 = t62+t63;
t65 = t5.*t7.*t11.*(2.1e1./2.0e2);
t84 = t10.*t11.*t12.*(2.1e1./2.0e2);
t66 = t53-t84;
t67 = t13.*t16;
t71 = t14.*t15.*t20;
t68 = t67-t71;
t69 = t57+t60;
t70 = t43+t44;
t72 = t9.*t18.*t19;
t73 = t27+t28+t43+t44;
t74 = t12.*t73.*(2.1e1./2.0e2);
t75 = t35-t45+t74;
t76 = t30-t32-t46+t72;
t77 = t12.*t76.*(2.1e1./2.0e2);
t78 = t34-t49+t77;
t79 = t11.*t12.*t18.*t64.*(2.1e1./2.0e2);
t83 = t5.*t11.*(2.1e1./2.0e2);
t80 = t50+t52-t83;
t81 = t54+t55+t56;
t82 = t58+t59+t61;
t85 = -t57+t58+t59-t60+t61;
t86 = t46-t72;
t87 = t33+t34-t49;
t88 = t53+t54+t55+t56-t84;
t89 = t50-t51+t52+t65-t83;
t90 = t51-t65;
dpg = [dxf-dphif.*(t47+t23.*t41+t26.*t48+t23.*t75+t16.*t24.*t78)+dthetaf.*(-t23.*t81+t26.*t80+t16.*t20.*t37-t16.*t24.*t82+t13.*t16.*t24.*t41+t15.*t16.*t24.*(t33+t34-t49-2.3e1./2.0e2))+dphit.*(t23.*t37+t16.*t24.*(t33+t34-t9.*t10.*4.0))+dthetat.*(-t26.*(t50-t51+t52+t65-t5.*t11.*(2.1e1./2.0e2))+t23.*(t53+t54+t55+t56-t10.*t11.*t12.*(2.1e1./2.0e2))+t16.*t24.*t85)-dpsif.*(-t41.*t64+t48.*t68+t14.*t24.*t37)-dthetag.*(t23.*t66+t26.*(t51-t5.*t7.*t11.*(2.1e1./2.0e2))-t16.*t24.*t69)+dphig.*(t47+t12.*t23.*t70.*(2.1e1./2.0e2)-t12.*t16.*t24.*(t46-t9.*t18.*t19).*(2.1e1./2.0e2));dyf-dphit.*(t37.*t68-t14.*t24.*t87)+dpsif.*(t26.*t41-t23.*t48+t16.*t24.*t37)+dthetag.*(t66.*t68+t64.*t90+t14.*t24.*t69)+dthetat.*(t64.*t89-t68.*t88+t14.*t24.*t85)-dphig.*(t79+t12.*t68.*t70.*(2.1e1./2.0e2)+t12.*t14.*t24.*t86.*(2.1e1./2.0e2))+dthetaf.*(-t64.*t80+t68.*t81+t14.*t20.*t37-t14.*t24.*t82+t13.*t14.*t24.*t41+t14.*t15.*t24.*t48)+dphif.*(t79+t41.*t68+t68.*t75+t64.*(t33+t34-t49-2.3e1./2.0e2)-t14.*t24.*t78);dzf+dphit.*(t20.*t87-t15.*t24.*t37)-dphif.*(t20.*t78-t15.*t24.*t41+t13.*t24.*t48-t15.*t24.*t75+t11.*t12.*t13.*t18.*t24.*(2.1e1./2.0e2))+dthetaf.*(-t24.*t37-t20.*t82+t13.*t20.*t41+t15.*t20.*t48+t13.*t24.*t80+t15.*t24.*t81)-dphig.*(t12.*t20.*t86.*(2.1e1./2.0e2)+t12.*t15.*t24.*t70.*(2.1e1./2.0e2)-t11.*t12.*t13.*t18.*t24.*(2.1e1./2.0e2))+dthetag.*(t20.*t69+t15.*t24.*t66-t13.*t24.*t90)-dthetat.*(-t20.*t85+t13.*t24.*t89+t15.*t24.*t88)];
