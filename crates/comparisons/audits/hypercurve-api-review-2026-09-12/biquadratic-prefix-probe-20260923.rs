use hyperreal::{Real, RealSign};
fn main() {
    let v0 = (Real::from(3_i64) / Real::from(64_i64)).unwrap();
    if v0.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v0"); return; }
    let v1 = Real::from(7_i64);
    if v1.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v1"); return; }
    let v2 = v1.clone().sqrt().unwrap();
    if v2.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v2"); return; }
    let v3 = &v0 * &v2;
    if v3.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v3"); return; }
    let v4 = &v3 * Real::from(2_i64);
    if v4.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v4"); return; }
    let v5 = -&v4;
    if v5.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v5"); return; }
    let v6 = (Real::from(29_i64) / Real::from(32_i64)).unwrap();
    if v6.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v6"); return; }
    let v7 = &v5 + &v6;
    if v7.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v7"); return; }
    let v8 = (Real::from(-93_i64) / Real::from(64_i64)).unwrap();
    if v8.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v8"); return; }
    let v9 = &v3 + &v8;
    if v9.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v9"); return; }
    let v10 = (Real::from(-3_i64) / Real::from(64_i64)).unwrap();
    if v10.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v10"); return; }
    let v11 = &v10 * &v2;
    if v11.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v11"); return; }
    let v12 = (Real::from(31_i64) / Real::from(64_i64)).unwrap();
    if v12.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v12"); return; }
    let v13 = &v11 + &v12;
    if v13.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v13"); return; }
    let v14 = &v13 + &v3;
    if v14.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v14"); return; }
    let v15 = (Real::from(3_i64) / Real::from(32_i64)).unwrap();
    if v15.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v15"); return; }
    let v16 = &v15 * &v2;
    if v16.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v16"); return; }
    let v17 = &v16 + &v12;
    if v17.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v17"); return; }
    let v18 = &v17 + &v3;
    if v18.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v18"); return; }
    let v19 = -&v18;
    if v19.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v19"); return; }
    let v20 = &v14 + &v19;
    if v20.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v20"); return; }
    let v21 = &v7 * &v20;
    if v21.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v21"); return; }
    let v22 = (Real::from(1_i64) / Real::from(2_i64)).unwrap();
    if v22.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v22"); return; }
    let v23 = &v3 + &v22;
    if v23.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v23"); return; }
    let v24 = (Real::from(-61_i64) / Real::from(64_i64)).unwrap();
    if v24.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v24"); return; }
    let v25 = &v3 + &v24;
    if v25.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v25"); return; }
    let v26 = -&v25;
    if v26.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v26"); return; }
    let v27 = &v23 + &v26;
    if v27.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v27"); return; }
    let v28 = Real::one();
    if v28.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v28"); return; }
    let v29 = &v18 + &v28;
    if v29.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v29"); return; }
    let v30 = &v27 * &v29;
    if v30.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v30"); return; }
    let v31 = &v30 * Real::from(2_i64);
    if v31.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v31"); return; }
    let v32 = -&v31;
    if v32.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v32"); return; }
    let v33 = &v21 + &v32;
    if v33.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v33"); return; }
    let v34 = (Real::from(4_i64) / Real::from(3_i64)).unwrap();
    if v34.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v34"); return; }
    let v35 = &v27 * &v20;
    if v35.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v35"); return; }
    let v36 = Real::from(3).sqrt().unwrap();
    if v36.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v36"); return; }
    let v37 = &v35 * &v36;
    if v37.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v37"); return; }
    let v38 = &v37 * &v36;
    if v38.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v38"); return; }
    let v39 = &v34 * &v38;
    if v39.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v39"); return; }
    let v40 = -&v39;
    if v40.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v40"); return; }
    let v41 = &v33 + &v40;
    if v41.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v41"); return; }
    let v42 = &v9 * &v41;
    if v42.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v42"); return; }
    let v43 = &v7 * &v29;
    if v43.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v43"); return; }
    let v44 = (Real::from(4_i64) / Real::from(9_i64)).unwrap();
    if v44.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v44"); return; }
    let v45 = &v44 * &v38;
    if v45.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v45"); return; }
    let v46 = &v43 + &v45;
    if v46.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v46"); return; }
    let v47 = &v27 * &v46;
    if v47.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v47"); return; }
    let v48 = &v42 + &v47;
    if v48.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v48"); return; }
    let v49 = (Real::from(2_i64) / Real::from(3_i64)).unwrap();
    if v49.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v49"); return; }
    let v50 = &v27 * &v41;
    if v50.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v50"); return; }
    let v51 = &v50 * &v36;
    if v51.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v51"); return; }
    let v52 = &v51 * &v36;
    if v52.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v52"); return; }
    let v53 = &v49 * &v52;
    if v53.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v53"); return; }
    let v54 = &v48 + &v53;
    if v54.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v54"); return; }
    let v55 = &v18 * &v20;
    if v55.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v55"); return; }
    let v56 = &v55 * Real::from(2_i64);
    if v56.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v56"); return; }
    let v57 = &v20 * &v29;
    if v57.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v57"); return; }
    let v58 = &v57 * Real::from(2_i64);
    if v58.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v58"); return; }
    let v59 = &v56 + &v58;
    if v59.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v59"); return; }
    let v60 = &v20 * &v20;
    if v60.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v60"); return; }
    let v61 = &v60 * &v36;
    if v61.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v61"); return; }
    let v62 = &v61 * &v36;
    if v62.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v62"); return; }
    let v63 = &v34 * &v62;
    if v63.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v63"); return; }
    let v64 = &v59 + &v63;
    if v64.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v64"); return; }
    let v65 = &v7 * &v64;
    if v65.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v65"); return; }
    let v66 = &v18 * &v29;
    if v66.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v66"); return; }
    let v67 = &v66 * Real::from(2_i64);
    if v67.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v67"); return; }
    let v68 = &v44 * &v62;
    if v68.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v68"); return; }
    let v69 = -&v68;
    if v69.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v69"); return; }
    let v70 = &v67 + &v69;
    if v70.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v70"); return; }
    let v71 = &v27 * &v70;
    if v71.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v71"); return; }
    let v72 = &v71 * Real::from(2_i64);
    if v72.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v72"); return; }
    let v73 = -&v72;
    if v73.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v73"); return; }
    let v74 = &v65 + &v73;
    if v74.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v74"); return; }
    let v75 = &v27 * &v64;
    if v75.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v75"); return; }
    let v76 = &v75 * &v36;
    if v76.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v76"); return; }
    let v77 = &v76 * &v36;
    if v77.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v77"); return; }
    let v78 = &v34 * &v77;
    if v78.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v78"); return; }
    let v79 = -&v78;
    if v79.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v79"); return; }
    let v80 = &v74 + &v79;
    if v80.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v80"); return; }
    let v81 = &v54 + &v80;
    if v81.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v81"); return; }
    let v82 = (Real::from(-29_i64) / Real::from(32_i64)).unwrap();
    if v82.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v82"); return; }
    let v83 = &v4 + &v82;
    if v83.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v83"); return; }
    let v84 = &v18 * Real::from(2_i64);
    if v84.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v84"); return; }
    let v85 = -&v84;
    if v85.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v85"); return; }
    let v86 = Real::from(-2_i64);
    if v86.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v86"); return; }
    let v87 = &v85 + &v86;
    if v87.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v87"); return; }
    let v88 = &v87 * &v20;
    if v88.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v88"); return; }
    let v89 = -&v58;
    if v89.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v89"); return; }
    let v90 = &v88 + &v89;
    if v90.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v90"); return; }
    let v91 = -&v63;
    if v91.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v91"); return; }
    let v92 = &v90 + &v91;
    if v92.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v92"); return; }
    let v93 = &v83 * &v92;
    if v93.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v93"); return; }
    let v94 = &v87 * &v29;
    if v94.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v94"); return; }
    let v95 = &v94 + &v68;
    if v95.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v95"); return; }
    let v96 = &v27 * &v95;
    if v96.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v96"); return; }
    let v97 = &v96 * Real::from(2_i64);
    if v97.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v97"); return; }
    let v98 = &v93 + &v97;
    if v98.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v98"); return; }
    let v99 = &v27 * &v92;
    if v99.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v99"); return; }
    let v100 = &v99 * &v36;
    if v100.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v100"); return; }
    let v101 = &v100 * &v36;
    if v101.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v101"); return; }
    let v102 = &v34 * &v101;
    if v102.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v102"); return; }
    let v103 = &v98 + &v102;
    if v103.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v103"); return; }
    let v104 = &v81 + &v103;
    if v104.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v104"); return; }
    let v105 = &v7 * &v104;
    if v105.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v105"); return; }
    let v106 = &v9 * &v46;
    if v106.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v106"); return; }
    let v107 = (Real::from(2_i64) / Real::from(9_i64)).unwrap();
    if v107.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v107"); return; }
    let v108 = &v107 * &v52;
    if v108.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v108"); return; }
    let v109 = -&v108;
    if v109.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v109"); return; }
    let v110 = &v106 + &v109;
    if v110.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v110"); return; }
    let v111 = &v7 * &v70;
    if v111.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v111"); return; }
    let v112 = &v44 * &v77;
    if v112.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v112"); return; }
    let v113 = &v111 + &v112;
    if v113.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v113"); return; }
    let v114 = &v110 + &v113;
    if v114.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v114"); return; }
    let v115 = &v83 * &v95;
    if v115.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v115"); return; }
    let v116 = &v44 * &v101;
    if v116.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v116"); return; }
    let v117 = -&v116;
    if v117.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v117"); return; }
    let v118 = &v115 + &v117;
    if v118.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v118"); return; }
    let v119 = &v114 + &v118;
    if v119.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v119"); return; }
    let v120 = &v27 * &v119;
    if v120.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v120"); return; }
    let v121 = &v120 * Real::from(2_i64);
    if v121.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v121"); return; }
    let v122 = -&v121;
    if v122.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v122"); return; }
    let v123 = &v105 + &v122;
    if v123.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v123"); return; }
    let v124 = &v27 * &v104;
    if v124.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v124"); return; }
    let v125 = &v124 * &v36;
    if v125.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v125"); return; }
    let v126 = &v125 * &v36;
    if v126.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v126"); return; }
    let v127 = &v34 * &v126;
    if v127.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v127"); return; }
    let v128 = -&v127;
    if v128.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v128"); return; }
    let v129 = &v123 + &v128;
    if v129.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v129"); return; }
    let v130 = &v87 * &v41;
    if v130.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v130"); return; }
    let v131 = &v20 * &v46;
    if v131.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v131"); return; }
    let v132 = &v131 * Real::from(2_i64);
    if v132.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v132"); return; }
    let v133 = -&v132;
    if v133.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v133"); return; }
    let v134 = &v130 + &v133;
    if v134.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v134"); return; }
    let v135 = &v20 * &v41;
    if v135.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v135"); return; }
    let v136 = &v135 * &v36;
    if v136.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v136"); return; }
    let v137 = &v136 * &v36;
    if v137.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v137"); return; }
    let v138 = &v34 * &v137;
    if v138.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v138"); return; }
    let v139 = -&v138;
    if v139.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v139"); return; }
    let v140 = &v134 + &v139;
    if v140.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v140"); return; }
    let v141 = &v18 * &v64;
    if v141.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v141"); return; }
    let v142 = &v141 * Real::from(2_i64);
    if v142.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v142"); return; }
    let v143 = &v20 * &v70;
    if v143.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v143"); return; }
    let v144 = &v143 * Real::from(2_i64);
    if v144.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v144"); return; }
    let v145 = &v142 + &v144;
    if v145.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v145"); return; }
    let v146 = &v20 * &v64;
    if v146.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v146"); return; }
    let v147 = &v146 * &v36;
    if v147.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v147"); return; }
    let v148 = &v147 * &v36;
    if v148.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v148"); return; }
    let v149 = &v34 * &v148;
    if v149.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v149"); return; }
    let v150 = &v145 + &v149;
    if v150.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v150"); return; }
    let v151 = &v140 + &v150;
    if v151.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v151"); return; }
    let v152 = &v83 * &v151;
    if v152.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v152"); return; }
    let v153 = &v87 * &v46;
    if v153.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v153"); return; }
    let v154 = &v44 * &v137;
    if v154.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v154"); return; }
    let v155 = &v153 + &v154;
    if v155.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v155"); return; }
    let v156 = &v18 * &v70;
    if v156.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v156"); return; }
    let v157 = &v156 * Real::from(2_i64);
    if v157.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v157"); return; }
    let v158 = &v44 * &v148;
    if v158.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v158"); return; }
    let v159 = -&v158;
    if v159.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v159"); return; }
    let v160 = &v157 + &v159;
    if v160.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v160"); return; }
    let v161 = &v155 + &v160;
    if v161.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v161"); return; }
    let v162 = &v27 * &v161;
    if v162.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v162"); return; }
    let v163 = &v162 * Real::from(2_i64);
    if v163.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v163"); return; }
    let v164 = &v152 + &v163;
    if v164.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v164"); return; }
    let v165 = &v27 * &v151;
    if v165.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v165"); return; }
    let v166 = &v165 * &v36;
    if v166.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v166"); return; }
    let v167 = &v166 * &v36;
    if v167.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v167"); return; }
    let v168 = &v34 * &v167;
    if v168.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v168"); return; }
    let v169 = &v164 + &v168;
    if v169.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v169"); return; }
    let v170 = &v129 + &v169;
    if v170.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v170"); return; }
    let v171 = -&v170;
    if v171.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v171"); return; }
    let v172 = &v7 * &v46;
    if v172.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v172"); return; }
    let v173 = &v44 * &v52;
    if v173.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v173"); return; }
    let v174 = &v172 + &v173;
    if v174.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v174"); return; }
    let v175 = &v83 * &v70;
    if v175.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v175"); return; }
    let v176 = -&v112;
    if v176.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v176"); return; }
    let v177 = &v175 + &v176;
    if v177.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v177"); return; }
    let v178 = &v174 + &v177;
    if v178.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v178"); return; }
    let v179 = -&v27;
    if v179.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v179"); return; }
    let v180 = &v20 * Real::from(4_i64);
    if v180.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v180"); return; }
    let v181 = -&v180;
    if v181.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v181"); return; }
    let v182 = &v179 + &v181;
    if v182.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v182"); return; }
    let v183 = &v178 * &v182;
    if v183.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v183"); return; }
    let v184 = -&v183;
    if v184.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v184"); return; }
    let v185 = &v7 * &v41;
    if v185.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v185"); return; }
    let v186 = &v47 * Real::from(2_i64);
    if v186.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v186"); return; }
    let v187 = -&v186;
    if v187.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v187"); return; }
    let v188 = &v185 + &v187;
    if v188.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v188"); return; }
    let v189 = &v34 * &v52;
    if v189.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v189"); return; }
    let v190 = -&v189;
    if v190.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v190"); return; }
    let v191 = &v188 + &v190;
    if v191.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v191"); return; }
    let v192 = &v83 * &v64;
    if v192.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v192"); return; }
    let v193 = &v192 + &v72;
    if v193.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v193"); return; }
    let v194 = &v193 + &v78;
    if v194.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v194"); return; }
    let v195 = &v191 + &v194;
    if v195.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v195"); return; }
    let v196 = -&v9;
    if v196.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v196"); return; }
    let v197 = &v18 * Real::from(4_i64);
    if v197.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v197"); return; }
    let v198 = -&v197;
    if v198.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v198"); return; }
    let v199 = &v196 + &v198;
    if v199.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v199"); return; }
    let v200 = &v195 * &v199;
    if v200.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v200"); return; }
    let v201 = -&v200;
    if v201.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v201"); return; }
    let v202 = &v184 + &v201;
    if v202.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v202"); return; }
    let v203 = &v195 * &v182;
    if v203.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v203"); return; }
    let v204 = &v203 * &v36;
    if v204.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v204"); return; }
    let v205 = &v204 * &v36;
    if v205.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v205"); return; }
    let v206 = &v49 * &v205;
    if v206.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v206"); return; }
    let v207 = -&v206;
    if v207.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v207"); return; }
    let v208 = &v202 + &v207;
    if v208.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v208"); return; }
    let v209 = &v171 + &v208;
    if v209.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v209"); return; }
    let v210 = &v83 * &v29;
    if v210.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v210"); return; }
    let v211 = -&v45;
    if v211.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v211"); return; }
    let v212 = &v210 + &v211;
    if v212.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v212"); return; }
    let v213 = &v21 * Real::from(2_i64);
    if v213.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v213"); return; }
    let v214 = -&v213;
    if v214.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v214"); return; }
    let v215 = &v27 * &v87;
    if v215.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v215"); return; }
    let v216 = &v215 * Real::from(2_i64);
    if v216.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v216"); return; }
    let v217 = -&v216;
    if v217.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v217"); return; }
    let v218 = &v214 + &v217;
    if v218.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v218"); return; }
    let v219 = (Real::from(8_i64) / Real::from(3_i64)).unwrap();
    if v219.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v219"); return; }
    let v220 = &v219 * &v38;
    if v220.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v220"); return; }
    let v221 = &v218 + &v220;
    if v221.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v221"); return; }
    let v222 = &v83 * &v20;
    if v222.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v222"); return; }
    let v223 = &v222 + &v31;
    if v223.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v223"); return; }
    let v224 = &v223 + &v39;
    if v224.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v224"); return; }
    let v225 = &v221 + &v224;
    if v225.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v225"); return; }
    let v226 = -&v225;
    if v226.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v226"); return; }
    let v227 = &v9 * &v20;
    if v227.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v227"); return; }
    let v228 = &v227 * Real::from(4_i64);
    if v228.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v228"); return; }
    let v229 = &v27 * &v18;
    if v229.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v229"); return; }
    let v230 = &v229 * Real::from(4_i64);
    if v230.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v230"); return; }
    let v231 = &v228 + &v230;
    if v231.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v231"); return; }
    let v232 = &v231 + &v220;
    if v232.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v232"); return; }
    let v233 = &v226 + &v232;
    if v233.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v233"); return; }
    let v234 = &v55 * Real::from(4_i64);
    if v234.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v234"); return; }
    let v235 = &v20 * &v18;
    if v235.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v235"); return; }
    let v236 = &v235 * Real::from(4_i64);
    if v236.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v236"); return; }
    let v237 = &v234 + &v236;
    if v237.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v237"); return; }
    let v238 = &v219 * &v62;
    if v238.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v238"); return; }
    let v239 = &v237 + &v238;
    if v239.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v239"); return; }
    let v240 = &v233 + &v239;
    if v240.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v240"); return; }
    let v241 = &v212 * &v240;
    if v241.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v241"); return; }
    let v242 = -&v241;
    if v242.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v242"); return; }
    let v243 = &v7 * &v87;
    if v243.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v243"); return; }
    let v244 = (Real::from(8_i64) / Real::from(9_i64)).unwrap();
    if v244.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v244"); return; }
    let v245 = &v244 * &v38;
    if v245.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v245"); return; }
    let v246 = -&v245;
    if v246.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v246"); return; }
    let v247 = &v243 + &v246;
    if v247.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v247"); return; }
    let v248 = &v247 + &v212;
    if v248.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v248"); return; }
    let v249 = -&v248;
    if v249.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v249"); return; }
    let v250 = &v9 * &v18;
    if v250.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v250"); return; }
    let v251 = &v250 * Real::from(4_i64);
    if v251.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v251"); return; }
    let v252 = &v251 + &v246;
    if v252.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v252"); return; }
    let v253 = &v249 + &v252;
    if v253.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v253"); return; }
    let v254 = &v18 * &v18;
    if v254.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v254"); return; }
    let v255 = &v254 * Real::from(4_i64);
    if v255.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v255"); return; }
    let v256 = &v244 * &v62;
    if v256.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v256"); return; }
    let v257 = -&v256;
    if v257.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v257"); return; }
    let v258 = &v255 + &v257;
    if v258.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v258"); return; }
    let v259 = &v253 + &v258;
    if v259.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v259"); return; }
    let v260 = &v224 * &v259;
    if v260.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v260"); return; }
    let v261 = -&v260;
    if v261.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v261"); return; }
    let v262 = &v242 + &v261;
    if v262.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v262"); return; }
    let v263 = &v224 * &v240;
    if v263.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v263"); return; }
    let v264 = &v263 * &v36;
    if v264.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v264"); return; }
    let v265 = &v264 * &v36;
    if v265.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v265"); return; }
    let v266 = &v49 * &v265;
    if v266.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v266"); return; }
    let v267 = -&v266;
    if v267.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v267"); return; }
    let v268 = &v262 + &v267;
    if v268.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v268"); return; }
    let v269 = &v209 + &v268;
    if v269.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v269"); return; }
    let v270 = -&v234;
    if v270.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v270"); return; }
    let v271 = &v20 * &v87;
    if v271.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v271"); return; }
    let v272 = &v271 * Real::from(2_i64);
    if v272.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v272"); return; }
    let v273 = &v270 + &v272;
    if v273.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v273"); return; }
    let v274 = -&v238;
    if v274.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v274"); return; }
    let v275 = &v273 + &v274;
    if v275.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v275"); return; }
    let v276 = &v7 * &v275;
    if v276.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v276"); return; }
    let v277 = &v18 * &v87;
    if v277.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v277"); return; }
    let v278 = &v277 * Real::from(2_i64);
    if v278.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v278"); return; }
    let v279 = &v278 + &v256;
    if v279.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v279"); return; }
    let v280 = &v27 * &v279;
    if v280.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v280"); return; }
    let v281 = &v280 * Real::from(2_i64);
    if v281.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v281"); return; }
    let v282 = -&v281;
    if v282.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v282"); return; }
    let v283 = &v276 + &v282;
    if v283.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v283"); return; }
    let v284 = &v27 * &v275;
    if v284.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v284"); return; }
    let v285 = &v284 * &v36;
    if v285.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v285"); return; }
    let v286 = &v285 * &v36;
    if v286.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v286"); return; }
    let v287 = &v34 * &v286;
    if v287.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v287"); return; }
    let v288 = -&v287;
    if v288.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v288"); return; }
    let v289 = &v283 + &v288;
    if v289.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v289"); return; }
    let v290 = &v88 * Real::from(2_i64);
    if v290.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v290"); return; }
    let v291 = -&v290;
    if v291.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v291"); return; }
    let v292 = -&v272;
    if v292.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v292"); return; }
    let v293 = &v291 + &v292;
    if v293.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v293"); return; }
    let v294 = &v293 + &v238;
    if v294.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v294"); return; }
    let v295 = &v294 + &v64;
    if v295.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v295"); return; }
    let v296 = &v83 * &v295;
    if v296.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v296"); return; }
    let v297 = &v87 * &v87;
    if v297.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v297"); return; }
    let v298 = &v297 + &v257;
    if v298.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v298"); return; }
    let v299 = &v298 + &v70;
    if v299.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v299"); return; }
    let v300 = &v27 * &v299;
    if v300.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v300"); return; }
    let v301 = &v300 * Real::from(2_i64);
    if v301.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v301"); return; }
    let v302 = &v296 + &v301;
    if v302.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v302"); return; }
    let v303 = &v27 * &v295;
    if v303.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v303"); return; }
    let v304 = &v303 * &v36;
    if v304.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v304"); return; }
    let v305 = &v304 * &v36;
    if v305.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v305"); return; }
    let v306 = &v34 * &v305;
    if v306.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v306"); return; }
    let v307 = &v302 + &v306;
    if v307.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v307"); return; }
    let v308 = &v289 + &v307;
    if v308.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v308"); return; }
    let v309 = -&v308;
    if v309.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v309"); return; }
    let v310 = &v248 * &v20;
    if v310.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v310"); return; }
    let v311 = &v310 * Real::from(4_i64);
    if v311.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v311"); return; }
    let v312 = &v225 * &v18;
    if v312.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v312"); return; }
    let v313 = &v312 * Real::from(4_i64);
    if v313.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v313"); return; }
    let v314 = &v311 + &v313;
    if v314.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v314"); return; }
    let v315 = &v225 * &v20;
    if v315.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v315"); return; }
    let v316 = &v315 * &v36;
    if v316.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v316"); return; }
    let v317 = &v316 * &v36;
    if v317.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v317"); return; }
    let v318 = &v219 * &v317;
    if v318.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v318"); return; }
    let v319 = &v314 + &v318;
    if v319.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v319"); return; }
    let v320 = &v309 + &v319;
    if v320.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v320"); return; }
    let v321 = &v9 * &v239;
    if v321.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v321"); return; }
    let v322 = -&v321;
    if v322.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v322"); return; }
    let v323 = &v27 * &v258;
    if v323.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v323"); return; }
    let v324 = -&v323;
    if v324.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v324"); return; }
    let v325 = &v322 + &v324;
    if v325.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v325"); return; }
    let v326 = &v27 * &v239;
    if v326.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v326"); return; }
    let v327 = &v326 * &v36;
    if v327.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v327"); return; }
    let v328 = &v327 * &v36;
    if v328.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v328"); return; }
    let v329 = &v49 * &v328;
    if v329.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v329"); return; }
    let v330 = -&v329;
    if v330.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v330"); return; }
    let v331 = &v325 + &v330;
    if v331.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v331"); return; }
    let v332 = &v320 + &v331;
    if v332.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v332"); return; }
    let v333 = &v9 * &v332;
    if v333.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v333"); return; }
    let v334 = -&v333;
    if v334.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v334"); return; }
    let v335 = &v7 * &v279;
    if v335.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v335"); return; }
    let v336 = &v44 * &v286;
    if v336.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v336"); return; }
    let v337 = &v335 + &v336;
    if v337.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v337"); return; }
    let v338 = &v83 * &v299;
    if v338.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v338"); return; }
    let v339 = &v44 * &v305;
    if v339.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v339"); return; }
    let v340 = -&v339;
    if v340.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v340"); return; }
    let v341 = &v338 + &v340;
    if v341.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v341"); return; }
    let v342 = &v337 + &v341;
    if v342.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v342"); return; }
    let v343 = -&v342;
    if v343.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v343"); return; }
    let v344 = &v248 * &v18;
    if v344.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v344"); return; }
    let v345 = &v344 * Real::from(4_i64);
    if v345.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v345"); return; }
    let v346 = &v244 * &v317;
    if v346.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v346"); return; }
    let v347 = -&v346;
    if v347.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v347"); return; }
    let v348 = &v345 + &v347;
    if v348.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v348"); return; }
    let v349 = &v343 + &v348;
    if v349.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v349"); return; }
    let v350 = &v9 * &v258;
    if v350.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v350"); return; }
    let v351 = -&v350;
    if v351.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v351"); return; }
    let v352 = &v107 * &v328;
    if v352.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v352"); return; }
    let v353 = &v351 + &v352;
    if v353.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v353"); return; }
    let v354 = &v349 + &v353;
    if v354.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v354"); return; }
    let v355 = &v27 * &v354;
    if v355.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v355"); return; }
    let v356 = -&v355;
    if v356.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v356"); return; }
    let v357 = &v334 + &v356;
    if v357.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v357"); return; }
    let v358 = &v27 * &v332;
    if v358.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v358"); return; }
    let v359 = &v358 * &v36;
    if v359.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v359"); return; }
    let v360 = &v359 * &v36;
    if v360.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v360"); return; }
    let v361 = &v49 * &v360;
    if v361.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v361"); return; }
    let v362 = -&v361;
    if v362.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v362"); return; }
    let v363 = &v357 + &v362;
    if v363.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v363"); return; }
    let v364 = &v269 + &v363;
    if v364.quadratic_tower_sign().is_none() { println!("first unresolved prefix: v364"); return; }
    let value = &v364 * ((Real::from(1_i64) / Real::from(1_i64)).unwrap());
    println!("shared replay tower={:?}", value.quadratic_tower_sign());
    assert_eq!(value.quadratic_tower_sign(), Some(RealSign::Zero));
}
