function out1 = get_C312017_09_27(in1,in2)
%GET_C312017_09_27
%    OUT1 = GET_C312017_09_27(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 5.10.
%    27-Sep-2017 15:20:32

dphif = in2(4,:);
dphig = in2(9,:);
dphit = in2(7,:);
dpsif = in2(6,:);
dpsig = in2(11,:);
dthetaa = in2(12,:);
dthetaf = in2(5,:);
dthetag = in2(10,:);
dthetat = in2(8,:);
phif = in1(4,:);
phig = in1(9,:);
phit = in1(7,:);
psif = in1(6,:);
psig = in1(11,:);
thetaa = in1(12,:);
thetaf = in1(5,:);
thetag = in1(10,:);
thetat = in1(8,:);
t2 = phif-phig;
t3 = sin(t2);
t4 = thetaf-thetat;
t5 = pi.*(1.0./2.0);
t6 = t5+thetag-thetat;
t7 = thetaa-thetag;
t8 = cos(t4);
t9 = cos(t6);
t10 = sin(t4);
t11 = sin(t6);
t12 = psif-psig;
t13 = sin(phif);
t14 = sin(psif);
t15 = cos(phif);
t16 = cos(psif);
t17 = -t5+thetaf;
t18 = cos(t17);
t19 = cos(t7);
t20 = cos(t12);
t21 = cos(t2);
t22 = phif-phit;
t23 = cos(t22);
t24 = sin(t12);
t25 = sin(t7);
t26 = sin(t22);
t27 = t21.*t26;
t28 = t3.*t8.*t23;
t29 = t27+t28;
t30 = t13.*t14;
t31 = t15.*t16.*t18;
t32 = t30+t31;
t33 = t14.*t15;
t181 = t13.*t16.*t18;
t34 = t33-t181;
t35 = t21.*t23;
t38 = t3.*t8.*t26;
t36 = t35-t38;
t37 = sin(t17);
t39 = t10.*t21.*t24.*t26;
t40 = t3.*t8.*t11;
t41 = t3.*t9.*t10;
t42 = t40+t41;
t43 = t25.*t42.*(1.1e1./1.25e2);
t44 = t3.*t8.*t9;
t96 = t3.*t10.*t11;
t45 = t44-t96;
t46 = t20.*t45;
t47 = t8.*t21.*t24;
t48 = t11.*t20.*t29;
t49 = t3.*t9.*t10.*t20.*t23;
t63 = t10.*t21.*t23.*t24;
t50 = t48+t49-t63;
t51 = t19.*t50.*(1.1e1./1.25e2);
t52 = t9.*t25.*t29.*(1.1e1./1.25e2);
t99 = t3.*t10.*t11.*t23.*t25.*(1.1e1./1.25e2);
t53 = t51+t52-t99;
t54 = t13.*t16;
t100 = t14.*t15.*t18;
t55 = t54-t100;
t56 = t9.*t29;
t91 = t3.*t10.*t11.*t23;
t57 = t56-t91;
t58 = t25.*t57.*(1.1e1./1.25e2);
t59 = t11.*t29;
t60 = t3.*t9.*t10.*t23;
t61 = t59+t60;
t62 = t20.*t61;
t64 = t15.*t16;
t65 = t13.*t14.*t18;
t66 = t64+t65;
t67 = t3.*t10.*t11.*t20;
t95 = t3.*t8.*t9.*t20;
t68 = t47+t67-t95;
t69 = t19.*t68.*(1.1e1./1.25e2);
t70 = t3.*t8.*t11.*t25.*(1.1e1./1.25e2);
t71 = t3.*t9.*t10.*t25.*(1.1e1./1.25e2);
t72 = t69+t70+t71;
t73 = t11.*t36;
t90 = t3.*t9.*t10.*t26;
t74 = t73-t90;
t75 = t20.*t74;
t76 = t39+t75;
t77 = t19.*t76.*(1.1e1./1.25e2);
t78 = t9.*t36;
t79 = t3.*t10.*t11.*t26;
t80 = t78+t79;
t81 = t25.*t80.*(1.1e1./1.25e2);
t82 = t77+t81;
t83 = t11.*t20.*t36;
t89 = t3.*t9.*t10.*t20.*t26;
t84 = t39+t83-t89;
t85 = t19.*t84.*(1.1e1./1.25e2);
t86 = t9.*t25.*t36.*(1.1e1./1.25e2);
t87 = t3.*t10.*t11.*t25.*t26.*(1.1e1./1.25e2);
t88 = t85+t86+t87;
t92 = t62-t63;
t93 = t19.*t92.*(1.1e1./1.25e2);
t94 = t58+t93;
t97 = t46-t47;
t366 = t19.*t97.*(1.1e1./1.25e2);
t98 = t43-t366;
t101 = t3.*t26.*2.0;
t102 = t3.*t8.*t26.*2.0;
t124 = t21.*t23.*2.0;
t125 = t8.*t21.*t23.*2.0;
t103 = t101+t102-t124-t125;
t104 = t3.*t26;
t107 = t8.*t21.*t23;
t105 = t104-t107;
t106 = t8.*t23.*4.0;
t108 = t9.*t10.*t23;
t109 = t10.*t21.*t24;
t138 = t3.*t9.*t10.*t20;
t110 = t109-t138;
t111 = t19.*t110.*(1.1e1./1.25e2);
t112 = t3.*t10.*t11.*t25.*(1.1e1./1.25e2);
t113 = t111+t112;
t114 = t3.*t23;
t115 = t8.*t21.*t26;
t116 = t27+t28+t114+t115;
t117 = t3.*t10.*t24;
t118 = t21.*t26.*2.0;
t119 = t3.*t23.*2.0;
t120 = t8.*t21.*t26.*2.0;
t121 = t3.*t8.*t23.*2.0;
t122 = t118+t119+t120+t121;
t123 = t10.*t11.*t26;
t126 = t8.*t26.*4.0;
t127 = t114+t115;
t128 = t24.*t127;
t129 = t9.*t20.*t36;
t130 = t128+t129;
t131 = t19.*t130.*(1.1e1./1.25e2);
t183 = t11.*t25.*t36.*(1.1e1./1.25e2);
t132 = t131-t183;
t133 = t24.*t105;
t134 = t9.*t20.*t29;
t135 = t133+t134;
t136 = t19.*t135.*(1.1e1./1.25e2);
t182 = t11.*t25.*t29.*(1.1e1./1.25e2);
t137 = t136-t182;
t139 = t9.*t122;
t140 = t123+t139;
t141 = t20.*t140;
t142 = t24.*t103;
t143 = t141+t142;
t144 = t11.*t122;
t171 = t9.*t10.*t26;
t145 = t144-t171;
t146 = t25.*t145.*(1.1e1./1.25e2);
t199 = t19.*t143.*(1.1e1./1.25e2);
t147 = t126+t146-t199;
t148 = t11.*t103;
t149 = t108+t148;
t150 = t9.*t103;
t156 = t10.*t11.*t23;
t151 = t150-t156;
t152 = t24.*t122;
t185 = t20.*t151;
t153 = t152-t185;
t154 = t19.*t153.*(1.1e1./1.25e2);
t155 = t9.*t105;
t157 = t24.*t29;
t158 = t11.*t105;
t159 = t108+t158;
t160 = t25.*t159.*(1.1e1./1.25e2);
t161 = t9.*t10.*t20.*t21;
t162 = t117+t161;
t163 = t19.*t162.*(1.1e1./1.25e2);
t187 = t10.*t11.*t21.*t25.*(1.1e1./1.25e2);
t164 = t163-t187;
t165 = t9.*t116;
t166 = t123+t165;
t167 = t20.*t166;
t168 = t35-t38-t104+t107;
t192 = t24.*t168;
t169 = t167-t192;
t170 = t11.*t116;
t172 = t10.*4.0;
t173 = t8.*t9;
t174 = t10.*t11.*t21;
t175 = t173+t174;
t176 = t25.*t175.*(1.1e1./1.25e2);
t177 = t8.*t11;
t196 = t9.*t10.*t21;
t178 = t177-t196;
t197 = t20.*t178;
t179 = t117-t197;
t198 = t19.*t179.*(1.1e1./1.25e2);
t180 = t172+t176-t198;
t184 = t25.*t149.*(1.1e1./1.25e2);
t186 = -t106+t154+t184;
t188 = t155-t156;
t201 = t20.*t188;
t189 = t157-t201;
t190 = t19.*t189.*(1.1e1./1.25e2);
t191 = -t106+t160+t190;
t193 = t170-t171;
t194 = t25.*t193.*(1.1e1./1.25e2);
t200 = t19.*t169.*(1.1e1./1.25e2);
t195 = t126+t194-t200;
t202 = t34.*t113;
t203 = t16.*t37.*t132;
t234 = t32.*t137;
t204 = t202+t203-t234;
t205 = t55.*t137;
t206 = t14.*t37.*t132;
t236 = t66.*t113;
t207 = t205+t206-t236;
t208 = t19.*t175.*(1.1e1./1.25e2);
t209 = t25.*t110.*(1.1e1./1.25e2);
t233 = t3.*t10.*t11.*t19.*(1.1e1./1.25e2);
t210 = t209-t233;
t211 = t19.*t159.*(1.1e1./1.25e2);
t230 = t25.*t189.*(1.1e1./1.25e2);
t212 = t211-t230;
t213 = t25.*t169.*(1.1e1./1.25e2);
t214 = t19.*t193.*(1.1e1./1.25e2);
t215 = t213+t214;
t216 = t9.*t168;
t217 = t156+t216;
t218 = t20.*t217;
t219 = t24.*t116;
t220 = t218+t219;
t221 = t25.*t220.*(1.1e1./1.25e2);
t228 = t11.*t168;
t222 = t108-t228;
t229 = t19.*t222.*(1.1e1./1.25e2);
t223 = t221-t229;
t224 = t18.*t132;
t225 = t15.*t37.*t137;
t226 = t13.*t37.*t113;
t227 = t224+t225+t226;
t231 = t25.*(t117-t197).*(1.1e1./1.25e2);
t232 = t208+t231;
t235 = t34.*t164;
t237 = t66.*t164;
t238 = t9.*t20.*t168;
t239 = t219+t238;
t240 = t19.*t239.*(1.1e1./1.25e2);
t247 = t11.*t25.*t168.*(1.1e1./1.25e2);
t241 = t240-t247;
t246 = t9.*t20.*t116;
t242 = t192-t246;
t243 = t19.*t242.*(1.1e1./1.25e2);
t244 = t11.*t25.*t116.*(1.1e1./1.25e2);
t245 = t243+t244;
t248 = t10.*t21.*t26;
t249 = t3.*t10.*t23;
t250 = t248+t249;
t251 = t3.*t10.*t26;
t255 = t10.*t21.*t23;
t252 = t251-t255;
t253 = t8.*t11.*t23;
t254 = t10.*t23.*4.0;
t256 = t8.*t9.*t23;
t257 = t9.*t252;
t258 = t253+t257;
t259 = t24.*t250;
t307 = t20.*t258;
t260 = t259-t307;
t309 = t11.*t252;
t261 = t256-t309;
t262 = t25.*t261.*(1.1e1./1.25e2);
t308 = t19.*t260.*(1.1e1./1.25e2);
t263 = t254+t262-t308;
t264 = t25.*t222.*(1.1e1./1.25e2);
t265 = t19.*t220.*(1.1e1./1.25e2);
t266 = -t106+t264+t265;
t267 = t10.*t11.*t21.*t23;
t268 = t256+t267;
t269 = t25.*t268.*(1.1e1./1.25e2);
t303 = t9.*t10.*t21.*t23;
t270 = t253-t303;
t271 = t20.*t270;
t304 = t3.*t10.*t23.*t24;
t272 = t271-t304;
t273 = t19.*t272.*(1.1e1./1.25e2);
t274 = t254+t269+t273;
t275 = t9.*t250;
t300 = t8.*t11.*t26;
t276 = t275-t300;
t277 = t20.*t276;
t278 = t24.*t252;
t279 = t277+t278;
t280 = t10.*t26.*4.0;
t281 = t11.*t250;
t282 = t8.*t9.*t26;
t283 = t281+t282;
t284 = t25.*t283.*(1.1e1./1.25e2);
t306 = t19.*t279.*(1.1e1./1.25e2);
t285 = t280+t284-t306;
t286 = t47-t95;
t287 = t19.*t286.*(1.1e1./1.25e2);
t288 = t70+t287;
t289 = t9.*t10;
t302 = t8.*t11.*t21;
t290 = t289-t302;
t291 = t25.*t290.*(1.1e1./1.25e2);
t292 = t10.*t11;
t293 = t8.*t9.*t21;
t294 = t292+t293;
t295 = t20.*t294;
t296 = t3.*t8.*t24;
t297 = t295+t296;
t298 = t19.*t297.*(1.1e1./1.25e2);
t301 = t8.*4.0;
t299 = t291+t298-t301;
t305 = t55.*t299;
t310 = t14.*t18.*t266;
t311 = t13.*t14.*t37.*t191;
t312 = t13.*t14.*t37.*t113;
t313 = t34.*t288;
t314 = t9.*t10.*t21.*t26;
t315 = t60-t300+t314;
t316 = t20.*t315;
t317 = t3.*t10.*t24.*t26;
t318 = -t63+t316+t317;
t319 = t10.*t11.*t21.*t26;
t320 = t91+t282+t319;
t321 = t25.*t320.*(1.1e1./1.25e2);
t336 = t19.*t318.*(1.1e1./1.25e2);
t322 = t280+t321-t336;
t323 = t34.*t274;
t324 = t90+t253-t303;
t332 = t20.*t324;
t325 = t39+t304-t332;
t326 = -t79+t256+t267;
t327 = t25.*t326.*(1.1e1./1.25e2);
t333 = t19.*t325.*(1.1e1./1.25e2);
t328 = t254+t327-t333;
t329 = t16.*t18.*t266;
t330 = t13.*t16.*t37.*t191;
t331 = t13.*t16.*t37.*t113;
t334 = t13.*t37.*t274;
t335 = t13.*t37.*t288;
t337 = t13.*t18.*t113;
t338 = t15.*t37.*t299;
t339 = t13.*t18.*t191;
t340 = t24.*t178;
t341 = t3.*t10.*t20;
t342 = t340+t341;
t343 = t10.*t20.*t21;
t344 = t3.*t9.*t10.*t24;
t345 = t343+t344;
t346 = t24.*t166;
t347 = t20.*t168;
t348 = t346+t347;
t349 = t24.*t188;
t350 = t20.*t29;
t351 = t349+t350;
t352 = t20.*t116;
t354 = t24.*t217;
t353 = t352-t354;
t355 = t19.*t34.*t345.*(1.1e1./1.25e2);
t356 = t19.*t32.*t348.*(1.1e1./1.25e2);
t357 = t19.*t34.*t351.*(1.1e1./1.25e2);
t358 = t19.*t55.*t342.*(1.1e1./1.25e2);
t359 = t14.*t19.*t37.*t353.*(1.1e1./1.25e2);
t360 = t15.*t19.*t37.*t342.*(1.1e1./1.25e2);
t361 = t13.*t19.*t37.*t345.*(1.1e1./1.25e2);
t362 = t13.*t19.*t37.*t351.*(1.1e1./1.25e2);
t363 = t16.*t37.*t88;
t364 = t32.*t195;
t365 = t32.*t180;
t367 = t34.*(-t106+t160+t190);
t368 = t16.*t37.*(-t106+t264+t265);
t369 = t202+t364+t365+t367+t368;
t370 = t53.*t55;
t371 = t66.*t72;
t372 = t14.*t37.*t88;
t373 = t55.*t195;
t374 = t55.*t180;
t375 = t34.*t72;
t376 = t66.*(-t106+t160+t190);
t573 = t14.*t37.*t266;
t377 = t236+t373+t374+t376-t573;
t378 = t155-t156+t256+t267;
t379 = t25.*t378.*(1.1e1./1.25e2);
t380 = t108+t158+t253-t303;
t391 = t20.*t380;
t381 = t304-t391;
t392 = t19.*t381.*(1.1e1./1.25e2);
t382 = t254+t379-t392;
t383 = t34.*t382;
t384 = t34.*t98;
t385 = t177-t196+t289-t302;
t386 = t25.*t385.*(1.1e1./1.25e2);
t387 = t173+t174-t292-t293;
t393 = t20.*t387;
t388 = t296-t393;
t389 = t19.*t388.*(1.1e1./1.25e2);
t390 = -t301+t386+t389;
t394 = t170-t171+t275-t300;
t395 = t20.*t394;
t396 = t24.*(t251-t255);
t397 = t395+t396;
t398 = t19.*t397.*(1.1e1./1.25e2);
t399 = t123+t165-t281-t282;
t400 = t25.*t399.*(1.1e1./1.25e2);
t401 = -t280+t398+t400;
t402 = t108-t228+t253+t257;
t427 = t20.*t402;
t403 = t259-t427;
t404 = t19.*t403.*(1.1e1./1.25e2);
t405 = t156+t216-t256+t309;
t406 = t25.*t405.*(1.1e1./1.25e2);
t407 = -t254+t404+t406;
t408 = t13.*t37.*t72;
t409 = t66.*t382;
t410 = t66.*t98;
t411 = t60+t170-t171-t300+t314;
t412 = t20.*t411;
t413 = -t63+t317+t412;
t414 = t91-t123-t165+t282+t319;
t415 = t25.*t414.*(1.1e1./1.25e2);
t429 = t19.*t413.*(1.1e1./1.25e2);
t416 = t280+t415-t429;
t417 = t90+t108-t228+t253-t303;
t428 = t20.*t417;
t418 = t39+t304-t428;
t419 = t19.*t418.*(1.1e1./1.25e2);
t420 = t79+t156+t216-t256-t267;
t421 = t25.*t420.*(1.1e1./1.25e2);
t422 = -t254+t419+t421;
t423 = t18.*t82;
t424 = t15.*t37.*t94;
t425 = t18.*t266;
t426 = t13.*t37.*t191;
t430 = t13.*t37.*t382;
t431 = t32.*t390;
t432 = t226+t425+t426-t15.*t37.*t180-t15.*t37.*t195;
t433 = t15.*t37.*t390;
t434 = t3.*t10.*t11.*t19.*t20.*(1.1e1./1.25e2);
t435 = t71+t209-t233+t434;
t436 = t25.*t178.*(1.1e1./1.25e2);
t446 = t19.*t20.*t175.*(1.1e1./1.25e2);
t437 = t208+t231+t436-t446;
t438 = t25.*t188.*(1.1e1./1.25e2);
t439 = t19.*t20.*t159.*(1.1e1./1.25e2);
t440 = -t211+t230+t438+t439;
t444 = t25.*t166.*(1.1e1./1.25e2);
t445 = t19.*t20.*t193.*(1.1e1./1.25e2);
t441 = t213+t214-t444-t445;
t442 = t19.*t20.*(t108-t228).*(1.1e1./1.25e2);
t447 = t25.*t217.*(1.1e1./1.25e2);
t443 = t221-t229+t442-t447;
t448 = t9.*t127;
t449 = t123+t448;
t450 = t24.*t36;
t454 = t20.*t449;
t457 = t450-t454;
t451 = t19.*t457.*(1.1e1./1.25e2);
t455 = t11.*t127;
t452 = t171-t455;
t456 = t25.*t452.*(1.1e1./1.25e2);
t453 = t126+t451-t456;
t458 = t18.*t443;
t459 = t15.*t37.*t441;
t460 = t13.*t37.*t435;
t461 = t15.*t37.*t437;
t462 = t13.*t37.*t440;
t463 = t458+t459+t460+t461+t462;
t464 = t25.*t457.*(1.1e1./1.25e2);
t465 = t19.*t452.*(1.1e1./1.25e2);
t466 = t25.*t449.*(1.1e1./1.25e2);
t470 = t19.*t20.*t452.*(1.1e1./1.25e2);
t467 = t464+t465+t466-t470;
t468 = t34.*t435;
t469 = t34.*t440;
t471 = t55.*t437;
t472 = t55.*t441;
t473 = t14.*t37.*t443;
t619 = t66.*t435;
t620 = t66.*t440;
t474 = t471+t472+t473-t619-t620;
t475 = t18.*t467;
t476 = t15.*t37.*t440;
t477 = t18.*t195;
t478 = t15.*t37.*(-t106+t264+t265);
t503 = t13.*t37.*t437;
t479 = t475+t476-t503;
t480 = t13.*t37.*t453;
t481 = t477+t478+t480;
t482 = t34.*t437;
t483 = t16.*t37.*t195;
t484 = t66.*t437;
t485 = t55.*t440;
t486 = t14.*t37.*t467;
t487 = t55.*t266;
t488 = t19.*(t450-t454).*(1.1e1./1.25e2);
t489 = t126-t456+t488;
t490 = t14.*t37.*t195;
t491 = t484+t485+t486;
t492 = t32.*(-t211+t230+t438+t439);
t511 = t16.*t37.*t467;
t493 = t482+t492-t511;
t494 = t32.*t186;
t495 = t32.*t191;
t496 = t32.*t113.*2.0;
t624 = t34.*t195.*2.0;
t625 = t34.*t180;
t626 = t16.*t37.*t147;
t497 = t235+t494+t495+t496-t624-t625-t626;
t498 = t55.*t186;
t499 = t55.*t191;
t500 = t55.*t113.*2.0;
t501 = t14.*t37.*t147;
t627 = t66.*t195.*2.0;
t628 = t66.*t180;
t502 = t237+t498+t499+t500+t501-t627-t628;
t504 = t18.*t147;
t505 = t15.*t37.*t113.*2.0;
t506 = t13.*t37.*t195.*2.0;
t507 = t13.*t37.*t180;
t508 = t15.*t37.*t186;
t509 = t15.*t37.*t191;
t550 = t13.*t37.*t164;
t510 = t504+t505+t506+t507+t508+t509-t550;
t512 = t34.*t212;
t513 = t32.*t215;
t514 = t55.*t232;
t515 = t66.*t212;
t516 = t55.*t215;
t517 = t14.*t37.*t223;
t632 = t66.*t210;
t518 = t514+t515+t516+t517-t632;
t519 = t18.*t223;
t520 = t15.*t37.*t215;
t521 = t15.*t37.*t232;
t522 = t13.*t37.*t210;
t629 = t13.*t37.*t212;
t523 = t519+t520+t521+t522-t629;
t524 = t32.*t401;
t525 = t375+t383-t431+t524-t16.*t37.*t407;
t526 = t32.*t416;
t527 = t16.*t37.*t422;
t528 = -t383-t384+t431+t526+t527;
t529 = t55.*t401;
t530 = t14.*t37.*t422;
t597 = t55.*t390;
t603 = t55.*t416;
t531 = t409+t410+t530-t597-t603;
t532 = t11.*(t251-t255);
t533 = t156+t216-t256+t532;
t534 = t25.*t533.*(1.1e1./1.25e2);
t535 = -t254+t404+t534;
t536 = t15.*t37.*t416;
t537 = t13.*t37.*t98;
t593 = t18.*t422;
t538 = t430+t433+t536+t537-t593;
t539 = t32.*t113;
t540 = t32.*t241;
t541 = t34.*t137;
t634 = t16.*t37.*t245;
t542 = t235+t539+t540+t541-t634;
t543 = t55.*t113;
t544 = t55.*t241;
t545 = t66.*t137;
t546 = t14.*t37.*t245;
t547 = t237+t543+t544+t545+t546;
t548 = t18.*t245;
t549 = t15.*t37.*t241;
t551 = t15.*t37.*t113;
t552 = t55.*t322;
t553 = t14.*t37.*t328;
t554 = t14.*t37.*t263;
t555 = t16.*t37.*t328;
t559 = t32.*t299;
t563 = t15.*t16.*t37.*t195;
t564 = t15.*t16.*t37.*t180;
t609 = t32.*t322;
t556 = t313+t323+t329+t330+t331+t555-t559-t563-t564-t609;
t557 = t18.*t328;
t558 = t15.*t37.*t322;
t560 = t277+t396;
t566 = t19.*t560.*(1.1e1./1.25e2);
t561 = t280+t284-t566;
t562 = t16.*t37.*t263;
t565 = t18.*t263;
t567 = t13.*t18.*(-t106+t160+t190);
t569 = t66.*t274;
t570 = t66.*t288;
t571 = t14.*t15.*t37.*t180;
t572 = t14.*t15.*t37.*t195;
t568 = t305+t310+t311+t312+t552+t553-t569-t570-t571-t572;
t574 = t16.*t19.*t37.*(t352-t354).*(1.1e1./1.25e2);
t575 = t34.*t191;
t576 = t19.*t66.*t345.*(1.1e1./1.25e2);
t577 = t16.*t37.*t266;
t578 = t19.*t55.*t348.*(1.1e1./1.25e2);
t579 = t19.*t66.*t351.*(1.1e1./1.25e2);
t580 = t18.*t19.*(t352-t354).*(1.1e1./1.25e2);
t584 = t15.*t19.*t37.*t348.*(1.1e1./1.25e2);
t581 = t360+t361+t362+t580-t584;
t605 = t19.*t32.*t342.*(1.1e1./1.25e2);
t582 = t355+t356+t357+t574-t605;
t583 = -t358-t359+t576+t578+t579;
t585 = t479.*t581.*2.0;
t652 = t18.*t535;
t653 = t15.*t37.*t401;
t586 = t408+t430+t433-t652-t653;
t587 = t24.*t449;
t588 = t20.*t36;
t589 = t587+t588;
t590 = t18.*t19.*t589.*(1.1e1./1.25e2);
t591 = t15.*t19.*t37.*t351.*(1.1e1./1.25e2);
t607 = t13.*t19.*t37.*t342.*(1.1e1./1.25e2);
t592 = t590+t591-t607;
t594 = t19.*t34.*t342.*(1.1e1./1.25e2);
t595 = t19.*t32.*t351.*(1.1e1./1.25e2);
t604 = t16.*t19.*t37.*t589.*(1.1e1./1.25e2);
t596 = t594+t595-t604;
t598 = t14.*t37.*t535;
t599 = t19.*t66.*t342.*(1.1e1./1.25e2);
t600 = t19.*t55.*t351.*(1.1e1./1.25e2);
t601 = t14.*t19.*t37.*t589.*(1.1e1./1.25e2);
t602 = t599+t600+t601;
t606 = t202+t358+t359+t364+t365+t575-t576+t577-t578-t579;
t608 = t581.*t592.*2.0;
t612 = t37.*t266;
t614 = t15.*t18.*t195;
t615 = t15.*t18.*t180;
t610 = t334+t335+t337+t338+t557+t558+t567-t612-t614-t615;
t662 = t32.*t561;
t611 = t313+t323+t329+t330+t331-t559+t562-t563-t564-t662;
t613 = t15.*t37.*t561;
t616 = t55.*t561;
t617 = t16.*t37.*t443;
t656 = t32.*t437;
t657 = t32.*t441;
t618 = t468+t469+t617-t656-t657;
t621 = t34.*t489;
t660 = t32.*t266;
t622 = t483+t621-t660;
t661 = t66.*t489;
t623 = t487+t490-t661;
t630 = t32.*t232;
t664 = t34.*t210;
t665 = t16.*t37.*t223;
t631 = t512+t513+t630-t664-t665;
t636 = t13.*t37.*t137;
t633 = t548+t549-t550+t551-t636;
t635 = t464+t465;
t637 = t32.*t212;
t638 = t16.*t37.*t635;
t639 = t236+t355+t356+t357+t373+t374+t376-t573+t574-t605;
t647 = t34.*t232;
t640 = t637+t638-t647;
t641 = t66.*t232;
t642 = t14.*t37.*t635;
t648 = t55.*t212;
t643 = t641+t642-t648;
t644 = t15.*t37.*t212;
t645 = t13.*t37.*t232;
t649 = t18.*t635;
t646 = t644+t645-t649;
t650 = t375+t383-t431+t524-t16.*t37.*t535;
t651 = t371+t409+t529-t597+t598;
t654 = t13.*t37.*(-t211+t230+t438+t439);
t655 = t458+t459+t460+t461+t654;
t658 = t13.*t37.*t489;
t659 = t477+t478+t658;
t663 = t334+t335+t337+t338+t565+t567-t612+t613-t614-t615;
out1 = reshape([dthetat.*(t363+t34.*(t43-t19.*(t46-t8.*t21.*t24).*(1.1e1./1.25e2))-t32.*t53-t34.*t72+t32.*(t58+t19.*(t62-t10.*t21.*t23.*t24).*(1.1e1./1.25e2))-t16.*t37.*t82),0.0,0.0,0.0,dthetat.*(t370+t371+t372-t55.*t94-t66.*t98-t14.*t37.*t82),0.0,0.0,0.0,-dthetat.*(t408+t423+t424-t18.*t88-t15.*t37.*t53-t13.*t37.*t98),0.0,0.0,0.0,dphit.*(t204.*(t483-t32.*t266+t34.*t453).*2.0+t227.*t481.*2.0+t207.*(t487+t490-t66.*t453).*2.0)-dpsif.*(t227.*(t360+t361+t362+t18.*t19.*t353.*(1.1e1./1.25e2)-t15.*t19.*t37.*t348.*(1.1e1./1.25e2)).*2.0+t207.*(t202+t358+t359+t364+t365+t575+t577-t19.*t55.*t348.*(1.1e1./1.25e2)-t19.*t66.*t345.*(1.1e1./1.25e2)-t19.*t66.*t351.*(1.1e1./1.25e2)).*2.0+t204.*(t236+t355+t356+t357+t373+t374+t66.*t191-t14.*t37.*t266-t19.*t32.*t342.*(1.1e1./1.25e2)+t16.*t19.*t37.*t353.*(1.1e1./1.25e2)).*2.0)+dphig.*(t227.*(t548+t549+t551-t13.*t37.*t137-t13.*t37.*t164).*2.0-t204.*t542.*2.0+t207.*t547.*2.0)-dthetaf.*(t227.*(t334+t335+t337+t338+t339+t557+t558-t37.*t266-t15.*t18.*t180-t15.*t18.*t195)+t204.*t556+t207.*t568+t227.*(t334+t335+t337+t338+t339+t565-t37.*t266-t15.*t18.*t180-t15.*t18.*t195+t15.*t37.*t285)+t207.*(t305+t310+t311+t312+t554+t55.*t285-t66.*t274-t66.*t288-t14.*t15.*t37.*t180-t14.*t15.*t37.*t195)+t204.*(t313+t323+t329+t330+t331+t562-t32.*t285-t32.*t299-t15.*t16.*t37.*t180-t15.*t16.*t37.*t195))+dthetaa.*(t204.*(t512+t513-t34.*t210+t32.*(t208+t25.*t179.*(1.1e1./1.25e2))-t16.*t37.*t223).*-2.0+t207.*t518.*2.0+t227.*t523.*2.0)-dthetag.*(t204.*(t468+t469-t32.*t437-t32.*t441+t16.*t37.*(t221-t229-t447+t19.*t20.*t222.*(1.1e1./1.25e2))).*2.0+t207.*t474.*2.0+t227.*t463.*2.0)-dthetat.*(t432.*(-t408+t18.*t88+t15.*t37.*t53)-t204.*t525+t204.*t528+t207.*t531-t227.*t538-t377.*(t370+t371+t372)-t227.*(t408+t430+t433-t18.*t407-t15.*t37.*t401)+t207.*(t371+t409+t529-t55.*t390+t14.*t37.*t407)+t369.*(t384+t32.*t94-t16.*t37.*t82)+t377.*(t410+t55.*t94+t14.*t37.*t82)-t369.*(-t363+t375+t32.*t53)-t432.*(t423+t424-t13.*t37.*t98))-dphif.*(t204.*t497.*-2.0+t207.*t502.*2.0+t227.*t510.*2.0)+dpsig.*(t204.*t582.*2.0-t207.*t583.*2.0+t227.*t581.*2.0),-dpsif.*(t585+t491.*t606.*2.0-t493.*(t236+t355+t356+t357+t373+t374+t376-t573+t574-t19.*t32.*t342.*(1.1e1./1.25e2)).*2.0)-dthetag.*(t463.*t479.*2.0+t474.*t491.*2.0-t618.*(t482+t32.*t440-t16.*t37.*t467).*2.0)+dthetat.*(t479.*t538-t493.*t525+t493.*t528+t479.*t586-t531.*(t484+t485+t486)-(t484+t485+t486).*(t371+t409+t529+t598-t55.*t390))-dpsig.*(-t585+t491.*t583.*2.0+t493.*t582.*2.0)-dthetaf.*(t479.*(t334+t335+t337+t338+t565+t567+t613-t37.*t266-t15.*t18.*t180-t15.*t18.*t195)+t491.*(t305+t310+t311+t312+t554-t569-t570-t571-t572+t616)-t493.*t556+t491.*t568+t479.*t610-t493.*t611)-dphif.*(t479.*t510.*2.0+t493.*t497.*2.0+t491.*t502.*2.0)+dphig.*(t493.*t542.*2.0+t491.*t547.*2.0+t479.*t633.*2.0)+dphit.*(t479.*t481.*2.0+t491.*t623.*2.0-t493.*t622.*2.0)+dthetaa.*(t479.*t523.*2.0+t491.*t518.*2.0+t493.*t631.*2.0),dpsif.*(t608+t602.*t606.*2.0-t596.*t639.*2.0)+dpsig.*(-t608+t582.*t596.*2.0+t583.*t602.*2.0)+dthetaf.*(t602.*(t305+t310+t311+t312+t552+t553-t569-t570-t571-t572)+t602.*(t305+t310+t311+t312+t554-t569-t570-t571-t572+t616)-t556.*t596+t592.*t610-t596.*t611+t592.*t663)-dthetat.*(t528.*t596+t538.*t592-t531.*t602+t586.*t592-t596.*t650-t602.*t651)+dphif.*(t497.*t596.*2.0+t510.*t592.*2.0+t502.*t602.*2.0)-dphig.*(t542.*t596.*2.0+t547.*t602.*2.0+t592.*t633.*2.0)-dphit.*(t596.*t622.*-2.0+t602.*t623.*2.0+t592.*t659.*2.0)-dthetaa.*(t523.*t592.*2.0+t518.*t602.*2.0+t596.*t631.*2.0)+dthetag.*(t474.*t602.*2.0-t596.*t618.*2.0+t592.*t655.*2.0),dthetat.*(t646.*(t408+t430+t433-t652-t653)-t640.*t650+t646.*(t430+t536+t537-t593+t15.*t37.*(-t301+t386+t389))+(t637+t638-t647).*(-t383-t384+t431+t526+t527)+t531.*(t641+t642-t648)+t651.*(t641+t642-t648))+dthetaf.*(-t610.*t646-t646.*t663+(t637+t638-t647).*(t313+t323+t329+t330+t331+t555-t559-t563-t564-t609)+(t637+t638-t647).*(t313+t323+t329+t330+t331-t559+t562-t563-t564-t662)+(t641+t642-t648).*(t305+t310+t311+t312+t552+t553-t569-t570-t571-t572)+(t641+t642-t648).*(t305+t310+t311+t312+t554-t569-t570-t571-t572+t616))+dpsig.*(t582.*t640.*-2.0+t581.*t646.*2.0+t583.*(t641+t642-t648).*2.0)+dthetaa.*(t518.*t643.*-2.0+t523.*t646.*2.0+t631.*(t637+t638-t647).*2.0)-dphif.*(t497.*t640.*2.0-t502.*t643.*2.0+t510.*t646.*2.0)+dphig.*(t542.*t640.*2.0-t547.*t643.*2.0+t633.*t646.*2.0)-dphit.*(t622.*t640.*2.0+t623.*t643.*2.0-t646.*t659.*2.0)+dpsif.*(t581.*t646.*-2.0+t606.*t643.*2.0+t639.*t640.*2.0)+dthetag.*(t474.*t643.*2.0+t618.*t640.*2.0-t646.*t655.*2.0)],[4, 4]);