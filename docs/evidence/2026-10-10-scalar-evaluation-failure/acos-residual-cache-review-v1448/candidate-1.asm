
target/typed-api-prototype-v1437-acos-residual-targeted-cost/probe:     file format elf64-x86-64


Disassembly of section .text:

00000000000a1e10 <<hyperreal::computable::node::Computable>::approx_signal>:
   a1e10:	push   %rbp
   a1e11:	push   %r15
   a1e13:	push   %r14
   a1e15:	push   %r13
   a1e17:	push   %r12
   a1e19:	push   %rbx
   a1e1a:	sub    $0x3a8,%rsp
   a1e21:	mov    %ecx,%ebp
   a1e23:	mov    %rsi,%r13
   a1e26:	mov    %rdi,%rbx
   a1e29:	mov    (%rdx),%r14
   a1e2c:	test   %r14,%r14
   a1e2f:	je     a1e3e <<hyperreal::computable::node::Computable>::approx_signal+0x2e>
   a1e31:	movzbl 0x10(%r14),%eax
   a1e36:	test   %al,%al
   a1e38:	jne    a1fe0 <<hyperreal::computable::node::Computable>::approx_signal+0x1d0>
   a1e3e:	mov    %rdx,0x40(%rsp)
   a1e43:	data16 data16 data16 mov %fs:0x0,%rax
   a1e4f:	lea    -0x120(%rax),%r15
   a1e56:	movzbl -0x110(%rax),%eax
   a1e5d:	test   %eax,%eax
   a1e5f:	jne    a2de0 <<hyperreal::computable::node::Computable>::approx_signal+0xfd0>
   a1e65:	movabs $0x7fffffffffffffff,%rax
   a1e6f:	mov    (%r15),%rcx
   a1e72:	cmp    %rax,%rcx
   a1e75:	jae    a2e10 <<hyperreal::computable::node::Computable>::approx_signal+0x1000>
   a1e7b:	lea    0x1(%rcx),%rax
   a1e7f:	mov    %rax,(%r15)
   a1e82:	mov    0x8(%r15),%rdx
   a1e86:	test   %rdx,%rdx
   a1e89:	je     a1ea3 <<hyperreal::computable::node::Computable>::approx_signal+0x93>
   a1e8b:	nopl   0x0(%rax,%rax,1)
   a1e90:	cmpb   $0x0,0x20(%rdx)
   a1e94:	jne    a1fdd <<hyperreal::computable::node::Computable>::approx_signal+0x1cd>
   a1e9a:	mov    0x18(%rdx),%rdx
   a1e9e:	test   %rdx,%rdx
   a1ea1:	jne    a1e90 <<hyperreal::computable::node::Computable>::approx_signal+0x80>
   a1ea3:	mov    %rcx,(%r15)
   a1ea6:	mov    0x0(%r13),%r12
   a1eaa:	movzbl 0x10(%r12),%edx
   a1eb0:	test   %edx,%edx
   a1eb2:	jne    a1ed3 <<hyperreal::computable::node::Computable>::approx_signal+0xc3>
   a1eb4:	cmpb   $0x1,0x30(%r12)
   a1eba:	jne    a1ed3 <<hyperreal::computable::node::Computable>::approx_signal+0xc3>
   a1ebc:	movups -0x6d3f3(%rip),%xmm0        # 34ad0 <anon.7b75c566c6a805bace2b01974d435009.137.llvm.16730574089061123067+0x10>
   a1ec3:	movups %xmm0,0x10(%rbx)
   a1ec7:	movups -0x6d40e(%rip),%xmm0        # 34ac0 <anon.7b75c566c6a805bace2b01974d435009.137.llvm.16730574089061123067>
   a1ece:	jmp    a207e <<hyperreal::computable::node::Computable>::approx_signal+0x26e>
   a1ed3:	cmp    $0xfff00000,%ebp
   a1ed9:	jge    a1eec <<hyperreal::computable::node::Computable>::approx_signal+0xdc>
   a1edb:	neg    %ebp
   a1edd:	setno  %al
   a1ee0:	add    %al,%al
   a1ee2:	add    $0xa,%al
   a1ee4:	mov    %al,0x8(%rbx)
   a1ee7:	jmp    a1fe4 <<hyperreal::computable::node::Computable>::approx_signal+0x1d4>
   a1eec:	cmp    $0x2,%edx
   a1eef:	je     a2000 <<hyperreal::computable::node::Computable>::approx_signal+0x1f0>
   a1ef5:	cmp    $0x1,%edx
   a1ef8:	je     a1f5f <<hyperreal::computable::node::Computable>::approx_signal+0x14f>
   a1efa:	test   %edx,%edx
   a1efc:	jne    a202a <<hyperreal::computable::node::Computable>::approx_signal+0x21a>
   a1f02:	mov    %rax,(%r15)
   a1f05:	mov    0x8(%r15),%rdx
   a1f09:	test   %rdx,%rdx
   a1f0c:	je     a1f3a <<hyperreal::computable::node::Computable>::approx_signal+0x12a>
   a1f0e:	mov    %rdx,%rsi
   a1f11:	cmpb   $0x0,0x20(%rsi)
   a1f15:	jne    a1f50 <<hyperreal::computable::node::Computable>::approx_signal+0x140>
   a1f17:	mov    0x10(%rsi),%rdi
   a1f1b:	mov    0x18(%rdi),%rax
   a1f1f:	nop
   a1f20:	test   %rax,%rax
   a1f23:	je     a1f50 <<hyperreal::computable::node::Computable>::approx_signal+0x140>
   a1f25:	lea    -0x1(%rax),%r8
   a1f29:	lock cmpxchg %r8,0x18(%rdi)
   a1f2f:	jne    a1f20 <<hyperreal::computable::node::Computable>::approx_signal+0x110>
   a1f31:	mov    0x18(%rsi),%rsi
   a1f35:	test   %rsi,%rsi
   a1f38:	jne    a1f11 <<hyperreal::computable::node::Computable>::approx_signal+0x101>
   a1f3a:	mov    %rcx,(%r15)
   a1f3d:	add    $0x18,%r12
   a1f41:	mov    %rbx,%rdi
   a1f44:	mov    %r12,%rsi
   a1f47:	jmp    a1fb9 <<hyperreal::computable::node::Computable>::approx_signal+0x1a9>
   a1f49:	nopl   0x0(%rax)
   a1f50:	movb   $0x1,0x20(%rdx)
   a1f54:	mov    0x18(%rdx),%rdx
   a1f58:	test   %rdx,%rdx
   a1f5b:	jne    a1f50 <<hyperreal::computable::node::Computable>::approx_signal+0x140>
   a1f5d:	jmp    a1fdd <<hyperreal::computable::node::Computable>::approx_signal+0x1cd>
   a1f5f:	mov    %rax,(%r15)
   a1f62:	mov    0x8(%r15),%rdx
   a1f66:	test   %rdx,%rdx
   a1f69:	je     a1f9a <<hyperreal::computable::node::Computable>::approx_signal+0x18a>
   a1f6b:	mov    %rdx,%rsi
   a1f6e:	cmpb   $0x0,0x20(%rsi)
   a1f72:	jne    a1fd0 <<hyperreal::computable::node::Computable>::approx_signal+0x1c0>
   a1f74:	mov    0x10(%rsi),%rdi
   a1f78:	mov    0x18(%rdi),%rax
   a1f7c:	nopl   0x0(%rax)
   a1f80:	test   %rax,%rax
   a1f83:	je     a1fd0 <<hyperreal::computable::node::Computable>::approx_signal+0x1c0>
   a1f85:	lea    -0x1(%rax),%r8
   a1f89:	lock cmpxchg %r8,0x18(%rdi)
   a1f8f:	jne    a1f80 <<hyperreal::computable::node::Computable>::approx_signal+0x170>
   a1f91:	mov    0x18(%rsi),%rsi
   a1f95:	test   %rsi,%rsi
   a1f98:	jne    a1f6e <<hyperreal::computable::node::Computable>::approx_signal+0x15e>
   a1f9a:	mov    %rcx,(%r15)
   a1f9d:	lea    0x1160a4(%rip),%rdi        # 1b8048 <hyperreal::computable::node::signed::ONE+0x20>
   a1fa4:	lea    0x11607d(%rip),%r14        # 1b8028 <hyperreal::computable::node::signed::ONE>
   a1fab:	mov    %r14,%rsi
   a1fae:	call   60ca0 <<std::sync::once::Once>::call_once_force::<<std::sync::lazy_lock::LazyLock<num_bigint::bigint::BigInt>>::force::{closure#0}>>
   a1fb3:	mov    %rbx,%rdi
   a1fb6:	mov    %r14,%rsi
   a1fb9:	mov    %ebp,%edx
   a1fbb:	call   bc5f0 <hyperreal::computable::node::scale_at_precision_ref>
   a1fc0:	jmp    a1feb <<hyperreal::computable::node::Computable>::approx_signal+0x1db>
   a1fc2:	data16 data16 data16 data16 cs nopw 0x0(%rax,%rax,1)
   a1fd0:	movb   $0x1,0x20(%rdx)
   a1fd4:	mov    0x18(%rdx),%rdx
   a1fd8:	test   %rdx,%rdx
   a1fdb:	jne    a1fd0 <<hyperreal::computable::node::Computable>::approx_signal+0x1c0>
   a1fdd:	mov    %rcx,(%r15)
   a1fe0:	movb   $0xc,0x8(%rbx)
   a1fe4:	movq   $0xffffffffffffffff,(%rbx)
   a1feb:	mov    %rbx,%rax
   a1fee:	add    $0x3a8,%rsp
   a1ff5:	pop    %rbx
   a1ff6:	pop    %r12
   a1ff8:	pop    %r13
   a1ffa:	pop    %r14
   a1ffc:	pop    %r15
   a1ffe:	pop    %rbp
   a1fff:	ret
   a2000:	movzbl 0x11(%r12),%esi
   a2006:	lea    0x210(%rsp),%rdi
   a200e:	mov    %ebp,%edx
   a2010:	call   b06f0 <<hyperreal::computable::node::Computable>::cached_constant_at_precision>
   a2015:	cmpq   $0xffffffffffffffff,0x210(%rsp)
   a201e:	jne    a206a <<hyperreal::computable::node::Computable>::approx_signal+0x25a>
   a2020:	movl   $0x0,0xc(%rsp)
   a2028:	jmp    a208c <<hyperreal::computable::node::Computable>::approx_signal+0x27c>
   a202a:	mov    0x38(%r12),%rax
   a202f:	test   $0x800,%eax
   a2034:	jne    a204b <<hyperreal::computable::node::Computable>::approx_signal+0x23b>
   a2036:	mov    %r13,%rdi
   a2039:	mov    0x40(%rsp),%rsi
   a203e:	call   b7830 <<hyperreal::computable::node::Computable>::ensure_finite_real>
   a2043:	cmp    $0xff,%al
   a2045:	jne    a1ee4 <<hyperreal::computable::node::Computable>::approx_signal+0xd4>
   a204b:	lea    0x40(%r12),%rsi
   a2050:	lea    0x210(%rsp),%rdi
   a2058:	mov    %ebp,%edx
   a205a:	call   bae60 <<hyperreal::computable::node::EvaluationCache>::at_precision>
   a205f:	cmpq   $0xffffffffffffffff,0x210(%rsp)
   a2068:	je     a2086 <<hyperreal::computable::node::Computable>::approx_signal+0x276>
   a206a:	movups 0x210(%rsp),%xmm0
   a2072:	movups 0x220(%rsp),%xmm1
   a207a:	movups %xmm1,0x10(%rbx)
   a207e:	movups %xmm0,(%rbx)
   a2081:	jmp    a1feb <<hyperreal::computable::node::Computable>::approx_signal+0x1db>
   a2086:	mov    $0x1,%al
   a2088:	mov    %eax,0xc(%rsp)
   a208c:	lea    0x10(%r12),%rax
   a2091:	mov    %rax,0x38(%rsp)
   a2096:	movzbl (%rax),%eax
   a2099:	add    $0xffffffdc,%eax
   a209c:	cmp    $0x1a,%eax
   a209f:	ja     a213b <<hyperreal::computable::node::Computable>::approx_signal+0x32b>
   a20a5:	lea    -0x6e628(%rip),%rcx        # 33a84 <anon.7b75c566c6a805bace2b01974d435009.371.llvm.16730574089061123067+0xb18>
   a20ac:	movslq (%rcx,%rax,4),%rax
   a20b0:	add    %rcx,%rax
   a20b3:	jmp    *%rax
   a20b5:	movl   $0x1,0x30(%rsp)
   a20bd:	jmp    a20db <<hyperreal::computable::node::Computable>::approx_signal+0x2cb>
   a20bf:	movl   $0x0,0x30(%rsp)
   a20c7:	jmp    a20db <<hyperreal::computable::node::Computable>::approx_signal+0x2cb>
   a20c9:	movl   $0x2,0x30(%rsp)
   a20d1:	jmp    a20db <<hyperreal::computable::node::Computable>::approx_signal+0x2cb>
   a20d3:	movl   $0x3,0x30(%rsp)
   a20db:	lea    0x18(%r12),%rdi
   a20e0:	call   b73b0 <<hyperreal::computable::node::Computable>::half_pi_index_when_real>
   a20e5:	test   $0x1,%al
   a20e7:	je     a213b <<hyperreal::computable::node::Computable>::approx_signal+0x32b>
   a20e9:	movzbl %dl,%edi
   a20ec:	movl   $0xff000100,0x210(%rsp)
   a20f7:	cmp    $0x3,%dil
   a20fb:	ja     a2fc4 <<hyperreal::computable::node::Computable>::approx_signal+0x11b4>
   a2101:	movzbl 0x210(%rsp,%rdi,1),%ecx
   a2109:	movl   $0xff0001,0x210(%rsp)
   a2114:	movzbl 0x210(%rsp,%rdi,1),%eax
   a211c:	mov    0x30(%rsp),%edx
   a2120:	lea    -0x6e637(%rip),%rsi        # 33af0 <anon.7b75c566c6a805bace2b01974d435009.371.llvm.16730574089061123067+0xb84>
   a2127:	movslq (%rsi,%rdx,4),%rdx
   a212b:	add    %rsi,%rdx
   a212e:	jmp    *%rdx
   a2130:	mov    %ecx,%eax
   a2132:	movsbq %al,%rsi
   a2136:	jmp    a2b14 <<hyperreal::computable::node::Computable>::approx_signal+0xd04>
   a213b:	cmpb   $0x0,0xc(%rsp)
   a2140:	je     a2175 <<hyperreal::computable::node::Computable>::approx_signal+0x365>
   a2142:	lea    0x210(%rsp),%rdi
   a214a:	mov    %r13,%rsi
   a214d:	mov    0x40(%rsp),%rdx
   a2152:	mov    %ebp,%ecx
   a2154:	call   a1130 <<hyperreal::computable::node::Computable>::cached_quadratic_tower_approximation>
   a2159:	mov    0x210(%rsp),%r12
   a2161:	cmp    $0xffffffffffffffff,%r12
   a2165:	je     a2be2 <<hyperreal::computable::node::Computable>::approx_signal+0xdd2>
   a216b:	cmp    $0xfffffffffffffffe,%r12
   a216f:	jne    a2cd8 <<hyperreal::computable::node::Computable>::approx_signal+0xec8>
   a2175:	mov    0x38(%rsp),%rsi
   a217a:	movzbl (%rsi),%eax
   a217d:	cmp    $0xa,%eax
   a2180:	ja     a2bef <<hyperreal::computable::node::Computable>::approx_signal+0xddf>
   a2186:	mov    $0x430,%ecx
   a218b:	bt     %eax,%ecx
   a218e:	jae    a2bef <<hyperreal::computable::node::Computable>::approx_signal+0xddf>
   a2194:	movl   $0xffffffff,0x108(%rsp)
   a219f:	movl   $0xffffffff,0x118(%rsp)
   a21aa:	movl   $0xffffffff,0x128(%rsp)
   a21b5:	movl   $0xffffffff,0x138(%rsp)
   a21c0:	movl   $0xffffffff,0x148(%rsp)
   a21cb:	movl   $0xffffffff,0x158(%rsp)
   a21d6:	movl   $0xffffffff,0x168(%rsp)
   a21e1:	movl   $0xffffffff,0x178(%rsp)
   a21ec:	movl   $0xffffffff,0x188(%rsp)
   a21f7:	movl   $0xffffffff,0x198(%rsp)
   a2202:	movl   $0xffffffff,0x1a8(%rsp)
   a220d:	movl   $0xffffffff,0x1b8(%rsp)
   a2218:	movl   $0xffffffff,0x1c8(%rsp)
   a2223:	movl   $0xffffffff,0x1d8(%rsp)
   a222e:	movl   $0xffffffff,0x1e8(%rsp)
   a2239:	movl   $0xffffffff,0x1f8(%rsp)
   a2244:	movq   $0x0,0x208(%rsp)
   a2250:	movq   $0x0,0xf0(%rsp)
   a225c:	movq   $0x8,0xf8(%rsp)
   a2268:	movq   $0x0,0x100(%rsp)
   a2274:	movq   $0xffffffffffffffff,0x228(%rsp)
   a2280:	movq   $0xffffffffffffffff,0x248(%rsp)
   a228c:	movq   $0xffffffffffffffff,0x268(%rsp)
   a2298:	movq   $0xffffffffffffffff,0x288(%rsp)
   a22a4:	movq   $0xffffffffffffffff,0x2a8(%rsp)
   a22b0:	movq   $0xffffffffffffffff,0x2c8(%rsp)
   a22bc:	movq   $0xffffffffffffffff,0x2e8(%rsp)
   a22c8:	movq   $0xffffffffffffffff,0x308(%rsp)
   a22d4:	movq   $0x0,0x328(%rsp)
   a22e0:	movq   $0x0,0x210(%rsp)
   a22ec:	movq   $0x8,0x218(%rsp)
   a22f8:	movq   $0x0,0x220(%rsp)
   a2304:	mov    %r13,0x18(%rsp)
   a2309:	mov    %ebp,0x14(%rsp)
   a230d:	movl   $0x0,0x10(%rsp)
   a2315:	lea    0xf0(%rsp),%rdi
   a231d:	lea    0x10(%rsp),%rsi
   a2322:	call   a1810 <<hyperreal::computable::node::InlineStack<<hyperreal::computable::node::Computable>::approx_signal::Frame, 16>>::push>
   a2327:	lea    0xe0742(%rip),%rax        # 182a70 <anon.7b75c566c6a805bace2b01974d435009.258.llvm.16730574089061123067+0x30>
   a232e:	mov    %rax,0x38(%rsp)
   a2333:	lea    -0x6d9da(%rip),%rax        # 34960 <anon.7b75c566c6a805bace2b01974d435009.54.llvm.16730574089061123067+0x6c>
   a233a:	mov    %rax,0x30(%rsp)
   a233f:	mov    0x100(%rsp),%rax
   a2347:	test   %rax,%rax
   a234a:	je     a2373 <<hyperreal::computable::node::Computable>::approx_signal+0x563>
   a234c:	dec    %rax
   a234f:	mov    %rax,0x100(%rsp)
   a2357:	mov    0xf8(%rsp),%rcx
   a235f:	shl    $0x4,%rax
   a2363:	mov    (%rcx,%rax,1),%r12d
   a2367:	mov    0x4(%rcx,%rax,1),%r9d
   a236c:	mov    0x8(%rcx,%rax,1),%r13
   a2371:	jmp    a23bb <<hyperreal::computable::node::Computable>::approx_signal+0x5ab>
   a2373:	mov    0x208(%rsp),%rax
   a237b:	test   %rax,%rax
   a237e:	je     a2dac <<hyperreal::computable::node::Computable>::approx_signal+0xf9c>
   a2384:	lea    -0x1(%rax),%rdi
   a2388:	mov    %rdi,0x208(%rsp)
   a2390:	cmp    $0x10,%rax
   a2394:	ja     a2fd6 <<hyperreal::computable::node::Computable>::approx_signal+0x11c6>
   a239a:	shl    $0x4,%rdi
   a239e:	lea    0x108(%rsp),%rax
   a23a6:	mov    (%rax,%rdi,1),%r12d
   a23aa:	mov    0x4(%rax,%rdi,1),%r9d
   a23af:	mov    0x8(%rax,%rdi,1),%r13
   a23b4:	movl   $0xffffffff,(%rax,%rdi,1)
   a23bb:	cmp    $0xffffffff,%r12d
   a23bf:	je     a2dac <<hyperreal::computable::node::Computable>::approx_signal+0xf9c>
   a23c5:	test   %r14,%r14
   a23c8:	je     a23d7 <<hyperreal::computable::node::Computable>::approx_signal+0x5c7>
   a23ca:	movzbl 0x10(%r14),%eax
   a23cf:	test   %al,%al
   a23d1:	jne    a2ad0 <<hyperreal::computable::node::Computable>::approx_signal+0xcc0>
   a23d7:	movzbl 0x10(%r15),%eax
   a23dc:	test   %eax,%eax
   a23de:	jne    a2a66 <<hyperreal::computable::node::Computable>::approx_signal+0xc56>
   a23e4:	mov    (%r15),%rcx
   a23e7:	movabs $0x7fffffffffffffff,%rax
   a23f1:	cmp    %rax,%rcx
   a23f4:	jae    a2aa0 <<hyperreal::computable::node::Computable>::approx_signal+0xc90>
   a23fa:	lea    0x1(%rcx),%rax
   a23fe:	mov    %rax,(%r15)
   a2401:	mov    0x8(%r15),%rdx
   a2405:	test   %rdx,%rdx
   a2408:	je     a243e <<hyperreal::computable::node::Computable>::approx_signal+0x62e>
   a240a:	mov    %rdx,%rsi
   a240d:	cmpb   $0x0,0x20(%rsi)
   a2411:	jne    a2ac0 <<hyperreal::computable::node::Computable>::approx_signal+0xcb0>
   a2417:	mov    0x10(%rsi),%rdi
   a241b:	mov    0x18(%rdi),%rax
   a241f:	nop
   a2420:	test   %rax,%rax
   a2423:	je     a2ac0 <<hyperreal::computable::node::Computable>::approx_signal+0xcb0>
   a2429:	lea    -0x1(%rax),%r8
   a242d:	lock cmpxchg %r8,0x18(%rdi)
   a2433:	jne    a2420 <<hyperreal::computable::node::Computable>::approx_signal+0x610>
   a2435:	mov    0x18(%rsi),%rsi
   a2439:	test   %rsi,%rsi
   a243c:	jne    a240d <<hyperreal::computable::node::Computable>::approx_signal+0x5fd>
   a243e:	mov    %r9d,0xc(%rsp)
   a2443:	mov    %rcx,(%r15)
   a2446:	mov    %r12d,%eax
   a2449:	lea    -0x6e950(%rip),%rcx        # 33b00 <anon.7b75c566c6a805bace2b01974d435009.371.llvm.16730574089061123067+0xb94>
   a2450:	movslq (%rcx,%rax,4),%rax
   a2454:	add    %rcx,%rax
   a2457:	jmp    *%rax
   a2459:	mov    0x0(%r13),%rsi
   a245d:	cmpb   $0x0,0x10(%rsi)
   a2461:	jne    a2470 <<hyperreal::computable::node::Computable>::approx_signal+0x660>
   a2463:	cmpb   $0x1,0x30(%rsi)
   a2467:	lea    -0x6d9ae(%rip),%rax        # 34ac0 <anon.7b75c566c6a805bace2b01974d435009.137.llvm.16730574089061123067>
   a246e:	je     a24a4 <<hyperreal::computable::node::Computable>::approx_signal+0x694>
   a2470:	mov    0xc(%rsp),%edx
   a2474:	cmp    $0xffefffff,%edx
   a247a:	jle    a2ea5 <<hyperreal::computable::node::Computable>::approx_signal+0x1095>
   a2480:	lea    0x388(%rsp),%rdi
   a2488:	call   ae110 <<hyperreal::computable::node::Computable>::cached_at_precision>
   a248d:	cmpq   $0xffffffffffffffff,0x388(%rsp)
   a2496:	lea    0x388(%rsp),%rax
   a249e:	je     a2733 <<hyperreal::computable::node::Computable>::approx_signal+0x923>
   a24a4:	lea    0x210(%rsp),%rdi
   a24ac:	mov    %rax,%rsi
   a24af:	call   a1730 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::push>
   a24b4:	jmp    a233f <<hyperreal::computable::node::Computable>::approx_signal+0x52f>
   a24b9:	lea    0x10(%rsp),%rdi
   a24be:	lea    0x210(%rsp),%rsi
   a24c6:	call   a16a0 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::pop>
   a24cb:	mov    %r13,0x68(%rsp)
   a24d0:	mov    0x10(%rsp),%r12
   a24d5:	cmp    $0xffffffffffffffff,%r12
   a24d9:	je     a2f4b <<hyperreal::computable::node::Computable>::approx_signal+0x113b>
   a24df:	mov    0x18(%rsp),%r13
   a24e4:	lea    0x29(%rsp),%rax
   a24e9:	movups -0x9(%rax),%xmm0
   a24ed:	movaps %xmm0,0x350(%rsp)
   a24f5:	lea    0x10(%rsp),%rdi
   a24fa:	lea    0x210(%rsp),%rsi
   a2502:	call   a16a0 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::pop>
   a2507:	cmpq   $0xffffffffffffffff,0x10(%rsp)
   a250d:	je     a2f71 <<hyperreal::computable::node::Computable>::approx_signal+0x1161>
   a2513:	movups 0x10(%rsp),%xmm0
   a2518:	movups 0x20(%rsp),%xmm1
   a251d:	movaps %xmm1,0x370(%rsp)
   a2525:	movaps %xmm0,0x360(%rsp)
   a252d:	mov    %r12,0x10(%rsp)
   a2532:	mov    %r13,0x18(%rsp)
   a2537:	movaps 0x350(%rsp),%xmm0
   a253f:	lea    0x29(%rsp),%rax
   a2544:	movups %xmm0,-0x9(%rax)
   a2548:	lea    0x48(%rsp),%rbp
   a254d:	mov    %rbp,%rdi
   a2550:	lea    0x360(%rsp),%rsi
   a2558:	lea    0x10(%rsp),%rdx
   a255d:	call   c03c0 <<num_bigint::bigint::BigInt as core::ops::arith::Add>::add>
   a2562:	lea    0x330(%rsp),%r13
   a256a:	mov    %r13,%rdi
   a256d:	mov    %rbp,%rsi
   a2570:	mov    $0xfffffffe,%edx
   a2575:	call   bcad0 <hyperreal::computable::node::scale>
   a257a:	movzbl 0x348(%rsp),%r12d
   a2583:	mov    %rbp,%rdi
   a2586:	mov    %r13,%rsi
   a2589:	call   1297a0 <<alloc::vec::Vec<u64> as core::clone::Clone>::clone>
   a258e:	mov    0x58(%rsp),%rax
   a2593:	mov    %rax,0x20(%rsp)
   a2598:	movups 0x48(%rsp),%xmm0
   a259d:	movaps %xmm0,0x10(%rsp)
   a25a2:	mov    %r12b,0x28(%rsp)
   a25a7:	mov    0x68(%rsp),%rdi
   a25ac:	mov    %r14,%rsi
   a25af:	mov    0xc(%rsp),%edx
   a25b3:	lea    0x10(%rsp),%rcx
   a25b8:	call   adaa0 <<hyperreal::computable::node::Computable>::store_cache_value>
   a25bd:	cmp    $0xff,%al
   a25bf:	jne    a2e6e <<hyperreal::computable::node::Computable>::approx_signal+0x105e>
   a25c5:	lea    0x210(%rsp),%rdi
   a25cd:	mov    %r13,%rsi
   a25d0:	call   a1730 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::push>
   a25d5:	jmp    a233f <<hyperreal::computable::node::Computable>::approx_signal+0x52f>
   a25da:	lea    0x10(%rsp),%rdi
   a25df:	lea    0x210(%rsp),%rsi
   a25e7:	call   a16a0 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::pop>
   a25ec:	cmpq   $0xffffffffffffffff,0x10(%rsp)
   a25f2:	je     a2f2c <<hyperreal::computable::node::Computable>::approx_signal+0x111c>
   a25f8:	movups 0x10(%rsp),%xmm0
   a25fd:	movups 0x20(%rsp),%xmm1
   a2602:	movaps %xmm1,0xa0(%rsp)
   a260a:	movaps %xmm0,0x90(%rsp)
   a2612:	movzbl 0xa8(%rsp),%r12d
   a261b:	lea    0x48(%rsp),%rdi
   a2620:	lea    0x90(%rsp),%rsi
   a2628:	call   1297a0 <<alloc::vec::Vec<u64> as core::clone::Clone>::clone>
   a262d:	mov    0xc(%rsp),%edx
   a2631:	mov    0x58(%rsp),%rax
   a2636:	mov    %rax,0x20(%rsp)
   a263b:	movups 0x48(%rsp),%xmm0
   a2640:	movaps %xmm0,0x10(%rsp)
   a2645:	mov    %r12b,0x28(%rsp)
   a264a:	mov    %r13,%rdi
   a264d:	mov    %r14,%rsi
   a2650:	lea    0x10(%rsp),%rcx
   a2655:	call   adaa0 <<hyperreal::computable::node::Computable>::store_cache_value>
   a265a:	cmp    $0xff,%al
   a265c:	jne    a2e2a <<hyperreal::computable::node::Computable>::approx_signal+0x101a>
   a2662:	lea    0x210(%rsp),%rdi
   a266a:	lea    0x90(%rsp),%rsi
   a2672:	call   a1730 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::push>
   a2677:	jmp    a233f <<hyperreal::computable::node::Computable>::approx_signal+0x52f>
   a267c:	lea    0x10(%rsp),%rdi
   a2681:	lea    0x210(%rsp),%rsi
   a2689:	call   a16a0 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::pop>
   a268e:	cmpq   $0xffffffffffffffff,0x10(%rsp)
   a2694:	je     a2f6a <<hyperreal::computable::node::Computable>::approx_signal+0x115a>
   a269a:	mov    0x20(%rsp),%rax
   a269f:	mov    %rax,0x80(%rsp)
   a26a7:	movups 0x10(%rsp),%xmm0
   a26ac:	movaps %xmm0,0x70(%rsp)
   a26b1:	lea    0x29(%rsp),%rcx
   a26b6:	mov    (%rcx),%eax
   a26b8:	mov    $0x2,%r12b
   a26bb:	sub    0x28(%rsp),%r12b
   a26c0:	mov    0x3(%rcx),%ecx
   a26c3:	lea    0x89(%rsp),%rdx
   a26cb:	mov    %ecx,0x3(%rdx)
   a26ce:	mov    %eax,(%rdx)
   a26d0:	mov    %r12b,0x88(%rsp)
   a26d8:	lea    0x48(%rsp),%rdi
   a26dd:	lea    0x70(%rsp),%rsi
   a26e2:	call   1297a0 <<alloc::vec::Vec<u64> as core::clone::Clone>::clone>
   a26e7:	mov    0xc(%rsp),%edx
   a26eb:	mov    0x58(%rsp),%rax
   a26f0:	mov    %rax,0x20(%rsp)
   a26f5:	movups 0x48(%rsp),%xmm0
   a26fa:	movaps %xmm0,0x10(%rsp)
   a26ff:	mov    %r12b,0x28(%rsp)
   a2704:	mov    %r13,%rdi
   a2707:	mov    %r14,%rsi
   a270a:	lea    0x10(%rsp),%rcx
   a270f:	call   adaa0 <<hyperreal::computable::node::Computable>::store_cache_value>
   a2714:	cmp    $0xff,%al
   a2716:	jne    a2e4f <<hyperreal::computable::node::Computable>::approx_signal+0x103f>
   a271c:	lea    0x210(%rsp),%rdi
   a2724:	lea    0x70(%rsp),%rsi
   a2729:	call   a1730 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::push>
   a272e:	jmp    a233f <<hyperreal::computable::node::Computable>::approx_signal+0x52f>
   a2733:	lea    0x10(%rsp),%rdi
   a2738:	mov    %r13,%rsi
   a273b:	mov    0x40(%rsp),%rdx
   a2740:	mov    0xc(%rsp),%ecx
   a2744:	call   a1130 <<hyperreal::computable::node::Computable>::cached_quadratic_tower_approximation>
   a2749:	mov    0x10(%rsp),%r12
   a274e:	cmp    $0xfffffffffffffffe,%r12
   a2752:	je     a27ff <<hyperreal::computable::node::Computable>::approx_signal+0x9ef>
   a2758:	cmp    $0xffffffffffffffff,%r12
   a275c:	je     a2eb6 <<hyperreal::computable::node::Computable>::approx_signal+0x10a6>
   a2762:	mov    %r13,%rbp
   a2765:	movzbl 0x18(%rsp),%eax
   a276a:	lea    0x29(%rsp),%rdx
   a276f:	mov    -0x1(%rdx),%rcx
   a2773:	lea    0xb9(%rsp),%rsi
   a277b:	mov    %rcx,0xf(%rsi)
   a277f:	movups -0x10(%rdx),%xmm0
   a2783:	movups %xmm0,(%rsi)
   a2786:	mov    %r12,0xb0(%rsp)
   a278e:	mov    %al,0xb8(%rsp)
   a2795:	movzbl 0xc8(%rsp),%r13d
   a279e:	lea    0x48(%rsp),%rdi
   a27a3:	lea    0xb0(%rsp),%rsi
   a27ab:	call   1297a0 <<alloc::vec::Vec<u64> as core::clone::Clone>::clone>
   a27b0:	mov    0x58(%rsp),%rax
   a27b5:	mov    %rax,0x20(%rsp)
   a27ba:	movups 0x48(%rsp),%xmm0
   a27bf:	movaps %xmm0,0x10(%rsp)
   a27c4:	mov    %r13b,0x28(%rsp)
   a27c9:	mov    %rbp,%rdi
   a27cc:	mov    %r14,%rsi
   a27cf:	mov    0xc(%rsp),%edx
   a27d3:	lea    0x10(%rsp),%rcx
   a27d8:	call   adaa0 <<hyperreal::computable::node::Computable>::store_cache_value>
   a27dd:	cmp    $0xff,%al
   a27df:	jne    a2ec3 <<hyperreal::computable::node::Computable>::approx_signal+0x10b3>
   a27e5:	lea    0x210(%rsp),%rdi
   a27ed:	lea    0xb0(%rsp),%rsi
   a27f5:	call   a1730 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::push>
   a27fa:	jmp    a233f <<hyperreal::computable::node::Computable>::approx_signal+0x52f>
   a27ff:	mov    0x0(%r13),%r12
   a2803:	movzbl 0x10(%r12),%eax
   a2809:	cmp    $0xa,%eax
   a280c:	je     a2936 <<hyperreal::computable::node::Computable>::approx_signal+0xb26>
   a2812:	cmp    $0x5,%eax
   a2815:	je     a2877 <<hyperreal::computable::node::Computable>::approx_signal+0xa67>
   a2817:	cmp    $0x4,%eax
   a281a:	jne    a29a0 <<hyperreal::computable::node::Computable>::approx_signal+0xb90>
   a2820:	mov    %r13,0x18(%rsp)
   a2825:	mov    0xc(%rsp),%eax
   a2829:	mov    %eax,0x14(%rsp)
   a282d:	movl   $0x1,0x10(%rsp)
   a2835:	lea    0xf0(%rsp),%rdi
   a283d:	lea    0x10(%rsp),%rsi
   a2842:	call   a1810 <<hyperreal::computable::node::InlineStack<<hyperreal::computable::node::Computable>::approx_signal::Frame, 16>>::push>
   a2847:	add    $0x18,%r12
   a284b:	mov    %r12,0x18(%rsp)
   a2850:	mov    0xc(%rsp),%eax
   a2854:	mov    %eax,0x14(%rsp)
   a2858:	movl   $0x0,0x10(%rsp)
   a2860:	lea    0xf0(%rsp),%rdi
   a2868:	lea    0x10(%rsp),%rsi
   a286d:	call   a1810 <<hyperreal::computable::node::InlineStack<<hyperreal::computable::node::Computable>::approx_signal::Frame, 16>>::push>
   a2872:	jmp    a233f <<hyperreal::computable::node::Computable>::approx_signal+0x52f>
   a2877:	mov    %r13,0x18(%rsp)
   a287c:	mov    0xc(%rsp),%eax
   a2880:	mov    %eax,0x14(%rsp)
   a2884:	movl   $0x2,0x10(%rsp)
   a288c:	lea    0xf0(%rsp),%rdi
   a2894:	lea    0x10(%rsp),%rsi
   a2899:	call   a1810 <<hyperreal::computable::node::InlineStack<<hyperreal::computable::node::Computable>::approx_signal::Frame, 16>>::push>
   a289e:	lea    0x28(%r12),%rdx
   a28a3:	lea    0x18(%r12),%rbp
   a28a8:	mov    0x28(%r12),%rax
   a28ad:	mov    0x38(%rax),%r13
   a28b1:	shr    $0x10,%r13d
   a28b5:	mov    0x18(%r12),%rax
   a28ba:	mov    0x38(%rax),%r12
   a28be:	shr    $0x10,%r12d
   a28c2:	mov    0xc(%rsp),%ecx
   a28c6:	add    $0xfffffffe,%ecx
   a28c9:	cmp    %r12w,%r13w
   a28cd:	mov    %rdx,0x68(%rsp)
   a28d2:	mov    %rdx,%rax
   a28d5:	cmovg  %rbp,%rax
   a28d9:	mov    %rax,0x18(%rsp)
   a28de:	mov    %ecx,0xc(%rsp)
   a28e2:	mov    %ecx,0x14(%rsp)
   a28e6:	movl   $0x0,0x10(%rsp)
   a28ee:	lea    0xf0(%rsp),%rdi
   a28f6:	lea    0x10(%rsp),%rsi
   a28fb:	call   a1810 <<hyperreal::computable::node::InlineStack<<hyperreal::computable::node::Computable>::approx_signal::Frame, 16>>::push>
   a2900:	cmp    %r12w,%r13w
   a2904:	cmovg  0x68(%rsp),%rbp
   a290a:	mov    %rbp,0x18(%rsp)
   a290f:	mov    0xc(%rsp),%eax
   a2913:	mov    %eax,0x14(%rsp)
   a2917:	movl   $0x0,0x10(%rsp)
   a291f:	lea    0xf0(%rsp),%rdi
   a2927:	lea    0x10(%rsp),%rsi
   a292c:	call   a1810 <<hyperreal::computable::node::InlineStack<<hyperreal::computable::node::Computable>::approx_signal::Frame, 16>>::push>
   a2931:	jmp    a233f <<hyperreal::computable::node::Computable>::approx_signal+0x52f>
   a2936:	mov    %r13,0x18(%rsp)
   a293b:	mov    0xc(%rsp),%eax
   a293f:	mov    %eax,0x14(%rsp)
   a2943:	movl   $0x3,0x10(%rsp)
   a294b:	lea    0xf0(%rsp),%rdi
   a2953:	lea    0x10(%rsp),%rsi
   a2958:	call   a1810 <<hyperreal::computable::node::InlineStack<<hyperreal::computable::node::Computable>::approx_signal::Frame, 16>>::push>
   a295d:	mov    0xc(%rsp),%eax
   a2961:	sub    0x14(%r12),%eax
   a2966:	mov    %eax,0xc(%rsp)
   a296a:	jo     a2fbb <<hyperreal::computable::node::Computable>::approx_signal+0x11ab>
   a2970:	add    $0x18,%r12
   a2974:	mov    %r12,0x18(%rsp)
   a2979:	mov    0xc(%rsp),%eax
   a297d:	mov    %eax,0x14(%rsp)
   a2981:	movl   $0x0,0x10(%rsp)
   a2989:	lea    0xf0(%rsp),%rdi
   a2991:	lea    0x10(%rsp),%rsi
   a2996:	call   a1810 <<hyperreal::computable::node::InlineStack<<hyperreal::computable::node::Computable>::approx_signal::Frame, 16>>::push>
   a299b:	jmp    a233f <<hyperreal::computable::node::Computable>::approx_signal+0x52f>
   a29a0:	add    $0x10,%r12
   a29a4:	lea    0x10(%rsp),%rdi
   a29a9:	mov    %r12,%rsi
   a29ac:	mov    0x40(%rsp),%rdx
   a29b1:	mov    0xc(%rsp),%ecx
   a29b5:	call   ea480 <<hyperreal::computable::approximation::Approximation>::approximate>
   a29ba:	mov    0x10(%rsp),%r12
   a29bf:	movzbl 0x18(%rsp),%eax
   a29c4:	cmp    $0xffffffffffffffff,%r12
   a29c8:	je     a2f07 <<hyperreal::computable::node::Computable>::approx_signal+0x10f7>
   a29ce:	mov    %r13,%rbp
   a29d1:	lea    0x29(%rsp),%rdx
   a29d6:	mov    -0x1(%rdx),%rcx
   a29da:	lea    0xd9(%rsp),%rsi
   a29e2:	mov    %rcx,0xf(%rsi)
   a29e6:	movups -0x10(%rdx),%xmm0
   a29ea:	movups %xmm0,(%rsi)
   a29ed:	mov    %r12,0xd0(%rsp)
   a29f5:	mov    %al,0xd8(%rsp)
   a29fc:	movzbl 0xe8(%rsp),%r13d
   a2a05:	lea    0x48(%rsp),%rdi
   a2a0a:	lea    0xd0(%rsp),%rsi
   a2a12:	call   1297a0 <<alloc::vec::Vec<u64> as core::clone::Clone>::clone>
   a2a17:	mov    0x58(%rsp),%rax
   a2a1c:	mov    %rax,0x20(%rsp)
   a2a21:	movups 0x48(%rsp),%xmm0
   a2a26:	movaps %xmm0,0x10(%rsp)
   a2a2b:	mov    %r13b,0x28(%rsp)
   a2a30:	mov    %rbp,%rdi
   a2a33:	mov    %r14,%rsi
   a2a36:	mov    0xc(%rsp),%edx
   a2a3a:	lea    0x10(%rsp),%rcx
   a2a3f:	call   adaa0 <<hyperreal::computable::node::Computable>::store_cache_value>
   a2a44:	cmp    $0xff,%al
   a2a46:	jne    a2f0f <<hyperreal::computable::node::Computable>::approx_signal+0x10ff>
   a2a4c:	lea    0x210(%rsp),%rdi
   a2a54:	lea    0xd0(%rsp),%rsi
   a2a5c:	call   a1730 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::push>
   a2a61:	jmp    a233f <<hyperreal::computable::node::Computable>::approx_signal+0x52f>
   a2a66:	cmp    $0x2,%eax
   a2a69:	je     a2ef5 <<hyperreal::computable::node::Computable>::approx_signal+0x10e5>
   a2a6f:	mov    %r9d,%ebp
   a2a72:	mov    %r15,%rdi
   a2a75:	lea    0x876b4(%rip),%rsi        # 12a130 <std::sys::thread_local::native::eager::destroy::<core::cell::RefCell<core::option::Option<alloc::rc::Rc<hyperreal::computable::budget::ActiveBudget>>>>>
   a2a7c:	call   *0xe40fe(%rip)        # 186b80 <_DYNAMIC+0x3b0>
   a2a82:	movb   $0x0,0x10(%r15)
   a2a87:	mov    %ebp,%r9d
   a2a8a:	mov    (%r15),%rcx
   a2a8d:	movabs $0x7fffffffffffffff,%rax
   a2a97:	cmp    %rax,%rcx
   a2a9a:	jb     a23fa <<hyperreal::computable::node::Computable>::approx_signal+0x5ea>
   a2aa0:	lea    0xe1359(%rip),%rdi        # 183e00 <anon.953f3db8f5a996f677fb6a53175227a4.23.llvm.13083929455518204517>
   a2aa7:	call   *0xe40db(%rip)        # 186b88 <_DYNAMIC+0x3b8>
   a2aad:	jmp    a2fe8 <<hyperreal::computable::node::Computable>::approx_signal+0x11d8>
   a2ab2:	data16 data16 data16 data16 cs nopw 0x0(%rax,%rax,1)
   a2ac0:	movb   $0x1,0x20(%rdx)
   a2ac4:	mov    0x18(%rdx),%rdx
   a2ac8:	test   %rdx,%rdx
   a2acb:	jne    a2ac0 <<hyperreal::computable::node::Computable>::approx_signal+0xcb0>
   a2acd:	mov    %rcx,(%r15)
   a2ad0:	movb   $0xc,0x8(%rbx)
   a2ad4:	movq   $0xffffffffffffffff,(%rbx)
   a2adb:	lea    0x210(%rsp),%rdi
   a2ae3:	call   7dd70 <core::ptr::drop_glue::<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>>
   a2ae8:	mov    0xf0(%rsp),%rsi
   a2af0:	test   %rsi,%rsi
   a2af3:	je     a1feb <<hyperreal::computable::node::Computable>::approx_signal+0x1db>
   a2af9:	mov    0xf8(%rsp),%rdi
   a2b01:	shl    $0x4,%rsi
   a2b05:	jmp    a2bc1 <<hyperreal::computable::node::Computable>::approx_signal+0xdb1>
   a2b0a:	test   %al,%al
   a2b0c:	je     a2bd9 <<hyperreal::computable::node::Computable>::approx_signal+0xdc9>
   a2b12:	xor    %esi,%esi
   a2b14:	lea    0x10(%rsp),%rdi
   a2b19:	call   c0a30 <<num_bigint::bigint::BigInt as core::convert::From<i64>>::from>
   a2b1e:	add    $0x38,%r12
   a2b22:	mov    0x28(%rsp),%esi
   a2b26:	mov    %r12,%rdi
   a2b29:	call   bbb10 <<hyperreal::computable::node::AtomicFacts>::publish_exact_sign>
   a2b2e:	lea    0xf0(%rsp),%r15
   a2b36:	lea    0x10(%rsp),%rsi
   a2b3b:	mov    %r15,%rdi
   a2b3e:	mov    %ebp,%edx
   a2b40:	call   bc410 <hyperreal::computable::node::scale_at_precision>
   a2b45:	movzbl 0x108(%rsp),%r12d
   a2b4e:	lea    0x48(%rsp),%rdi
   a2b53:	mov    %r15,%rsi
   a2b56:	call   1297a0 <<alloc::vec::Vec<u64> as core::clone::Clone>::clone>
   a2b5b:	mov    0x58(%rsp),%rax
   a2b60:	mov    %rax,0x220(%rsp)
   a2b68:	movups 0x48(%rsp),%xmm0
   a2b6d:	movaps %xmm0,0x210(%rsp)
   a2b75:	mov    %r12b,0x228(%rsp)
   a2b7d:	lea    0x210(%rsp),%rcx
   a2b85:	mov    %r13,%rdi
   a2b88:	mov    %r14,%rsi
   a2b8b:	mov    %ebp,%edx
   a2b8d:	call   adaa0 <<hyperreal::computable::node::Computable>::store_cache_value>
   a2b92:	cmp    $0xff,%al
   a2b94:	je     a2d97 <<hyperreal::computable::node::Computable>::approx_signal+0xf87>
   a2b9a:	mov    %al,0x8(%rbx)
   a2b9d:	movq   $0xffffffffffffffff,(%rbx)
   a2ba4:	mov    0xf0(%rsp),%rsi
   a2bac:	test   %rsi,%rsi
   a2baf:	je     a1feb <<hyperreal::computable::node::Computable>::approx_signal+0x1db>
   a2bb5:	mov    0xf8(%rsp),%rdi
   a2bbd:	shl    $0x3,%rsi
   a2bc1:	mov    $0x8,%edx
   a2bc6:	call   *0xe3e44(%rip)        # 186a10 <_DYNAMIC+0x240>
   a2bcc:	jmp    a1feb <<hyperreal::computable::node::Computable>::approx_signal+0x1db>
   a2bd1:	test   %cl,%cl
   a2bd3:	jne    a2b12 <<hyperreal::computable::node::Computable>::approx_signal+0xd02>
   a2bd9:	movb   $0x2,0x8(%rbx)
   a2bdd:	jmp    a1fe4 <<hyperreal::computable::node::Computable>::approx_signal+0x1d4>
   a2be2:	movzbl 0x218(%rsp),%eax
   a2bea:	jmp    a1ee4 <<hyperreal::computable::node::Computable>::approx_signal+0xd4>
   a2bef:	lea    0x210(%rsp),%rdi
   a2bf7:	mov    0x40(%rsp),%rdx
   a2bfc:	mov    %ebp,%ecx
   a2bfe:	call   ea480 <<hyperreal::computable::approximation::Approximation>::approximate>
   a2c03:	mov    0x210(%rsp),%r15
   a2c0b:	movzbl 0x218(%rsp),%eax
   a2c13:	cmp    $0xffffffffffffffff,%r15
   a2c17:	je     a1ee4 <<hyperreal::computable::node::Computable>::approx_signal+0xd4>
   a2c1d:	mov    0x228(%rsp),%rcx
   a2c25:	mov    %rcx,0x108(%rsp)
   a2c2d:	movups 0x219(%rsp),%xmm0
   a2c35:	movups %xmm0,0xf9(%rsp)
   a2c3d:	mov    %r15,0xf0(%rsp)
   a2c45:	mov    %al,0xf8(%rsp)
   a2c4c:	movzbl 0x108(%rsp),%r12d
   a2c55:	lea    0x10(%rsp),%rdi
   a2c5a:	lea    0xf0(%rsp),%rsi
   a2c62:	call   1297a0 <<alloc::vec::Vec<u64> as core::clone::Clone>::clone>
   a2c67:	mov    0x20(%rsp),%rax
   a2c6c:	mov    %rax,0x220(%rsp)
   a2c74:	movups 0x10(%rsp),%xmm0
   a2c79:	movaps %xmm0,0x210(%rsp)
   a2c81:	mov    %r12b,0x228(%rsp)
   a2c89:	lea    0x210(%rsp),%rcx
   a2c91:	mov    %r13,%rdi
   a2c94:	mov    %r14,%rsi
   a2c97:	mov    %ebp,%edx
   a2c99:	call   adaa0 <<hyperreal::computable::node::Computable>::store_cache_value>
   a2c9e:	cmp    $0xff,%al
   a2ca0:	je     a2d97 <<hyperreal::computable::node::Computable>::approx_signal+0xf87>
   a2ca6:	mov    %al,0x8(%rbx)
   a2ca9:	movq   $0xffffffffffffffff,(%rbx)
   a2cb0:	test   %r15,%r15
   a2cb3:	je     a1feb <<hyperreal::computable::node::Computable>::approx_signal+0x1db>
   a2cb9:	mov    0xf8(%rsp),%rdi
   a2cc1:	shl    $0x3,%r15
   a2cc5:	mov    $0x8,%edx
   a2cca:	mov    %r15,%rsi
   a2ccd:	call   *0xe3d3d(%rip)        # 186a10 <_DYNAMIC+0x240>
   a2cd3:	jmp    a1feb <<hyperreal::computable::node::Computable>::approx_signal+0x1db>
   a2cd8:	movzbl 0x218(%rsp),%eax
   a2ce0:	mov    0x228(%rsp),%rcx
   a2ce8:	mov    %rcx,0x108(%rsp)
   a2cf0:	movups 0x219(%rsp),%xmm0
   a2cf8:	movups %xmm0,0xf9(%rsp)
   a2d00:	mov    %r12,0xf0(%rsp)
   a2d08:	mov    %al,0xf8(%rsp)
   a2d0f:	movzbl 0x108(%rsp),%r15d
   a2d18:	lea    0x10(%rsp),%rdi
   a2d1d:	lea    0xf0(%rsp),%rsi
   a2d25:	call   1297a0 <<alloc::vec::Vec<u64> as core::clone::Clone>::clone>
   a2d2a:	mov    0x20(%rsp),%rax
   a2d2f:	mov    %rax,0x220(%rsp)
   a2d37:	movups 0x10(%rsp),%xmm0
   a2d3c:	movaps %xmm0,0x210(%rsp)
   a2d44:	mov    %r15b,0x228(%rsp)
   a2d4c:	lea    0x210(%rsp),%rcx
   a2d54:	mov    %r13,%rdi
   a2d57:	mov    %r14,%rsi
   a2d5a:	mov    %ebp,%edx
   a2d5c:	call   adaa0 <<hyperreal::computable::node::Computable>::store_cache_value>
   a2d61:	cmp    $0xff,%al
   a2d63:	je     a2d97 <<hyperreal::computable::node::Computable>::approx_signal+0xf87>
   a2d65:	mov    %al,0x8(%rbx)
   a2d68:	movq   $0xffffffffffffffff,(%rbx)
   a2d6f:	test   %r12,%r12
   a2d72:	je     a1feb <<hyperreal::computable::node::Computable>::approx_signal+0x1db>
   a2d78:	mov    0xf8(%rsp),%rdi
   a2d80:	shl    $0x3,%r12
   a2d84:	mov    $0x8,%edx
   a2d89:	mov    %r12,%rsi
   a2d8c:	call   *0xe3c7e(%rip)        # 186a10 <_DYNAMIC+0x240>
   a2d92:	jmp    a1feb <<hyperreal::computable::node::Computable>::approx_signal+0x1db>
   a2d97:	movups 0xf0(%rsp),%xmm0
   a2d9f:	movups 0x100(%rsp),%xmm1
   a2da7:	jmp    a207a <<hyperreal::computable::node::Computable>::approx_signal+0x26a>
   a2dac:	lea    0x10(%rsp),%rdi
   a2db1:	lea    0x210(%rsp),%rsi
   a2db9:	call   a16a0 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::pop>
   a2dbe:	cmpq   $0xffffffffffffffff,0x10(%rsp)
   a2dc4:	je     a2f8c <<hyperreal::computable::node::Computable>::approx_signal+0x117c>
   a2dca:	movups 0x10(%rsp),%xmm0
   a2dcf:	movups 0x20(%rsp),%xmm1
   a2dd4:	movups %xmm1,0x10(%rbx)
   a2dd8:	movups %xmm0,(%rbx)
   a2ddb:	jmp    a2adb <<hyperreal::computable::node::Computable>::approx_signal+0xccb>
   a2de0:	cmp    $0x1,%eax
   a2de3:	jne    a2e1d <<hyperreal::computable::node::Computable>::approx_signal+0x100d>
   a2de5:	lea    0x87344(%rip),%rsi        # 12a130 <std::sys::thread_local::native::eager::destroy::<core::cell::RefCell<core::option::Option<alloc::rc::Rc<hyperreal::computable::budget::ActiveBudget>>>>>
   a2dec:	mov    %r15,%rdi
   a2def:	call   *0xe3d8b(%rip)        # 186b80 <_DYNAMIC+0x3b0>
   a2df5:	movb   $0x0,0x10(%r15)
   a2dfa:	movabs $0x7fffffffffffffff,%rax
   a2e04:	mov    (%r15),%rcx
   a2e07:	cmp    %rax,%rcx
   a2e0a:	jb     a1e7b <<hyperreal::computable::node::Computable>::approx_signal+0x6b>
   a2e10:	lea    0xe1001(%rip),%rdi        # 183e18 <anon.953f3db8f5a996f677fb6a53175227a4.24.llvm.13083929455518204517>
   a2e17:	call   *0xe3d6b(%rip)        # 186b88 <_DYNAMIC+0x3b8>
   a2e1d:	lea    0xe0f2c(%rip),%rdi        # 183d50 <anon.953f3db8f5a996f677fb6a53175227a4.4.llvm.13083929455518204517>
   a2e24:	call   *0xe3d66(%rip)        # 186b90 <_DYNAMIC+0x3c0>
   a2e2a:	mov    %al,0x8(%rbx)
   a2e2d:	movq   $0xffffffffffffffff,(%rbx)
   a2e34:	mov    0x90(%rsp),%rsi
   a2e3c:	test   %rsi,%rsi
   a2e3f:	je     a2adb <<hyperreal::computable::node::Computable>::approx_signal+0xccb>
   a2e45:	mov    0x98(%rsp),%rdi
   a2e4d:	jmp    a2e91 <<hyperreal::computable::node::Computable>::approx_signal+0x1081>
   a2e4f:	mov    %al,0x8(%rbx)
   a2e52:	movq   $0xffffffffffffffff,(%rbx)
   a2e59:	mov    0x70(%rsp),%rsi
   a2e5e:	test   %rsi,%rsi
   a2e61:	je     a2adb <<hyperreal::computable::node::Computable>::approx_signal+0xccb>
   a2e67:	mov    0x78(%rsp),%rdi
   a2e6c:	jmp    a2e91 <<hyperreal::computable::node::Computable>::approx_signal+0x1081>
   a2e6e:	mov    %al,0x8(%rbx)
   a2e71:	movq   $0xffffffffffffffff,(%rbx)
   a2e78:	mov    0x330(%rsp),%rsi
   a2e80:	test   %rsi,%rsi
   a2e83:	je     a2adb <<hyperreal::computable::node::Computable>::approx_signal+0xccb>
   a2e89:	mov    0x338(%rsp),%rdi
   a2e91:	shl    $0x3,%rsi
   a2e95:	mov    $0x8,%edx
   a2e9a:	call   *0xe3b70(%rip)        # 186a10 <_DYNAMIC+0x240>
   a2ea0:	jmp    a2adb <<hyperreal::computable::node::Computable>::approx_signal+0xccb>
   a2ea5:	neg    %edx
   a2ea7:	setno  %al
   a2eaa:	add    %al,%al
   a2eac:	add    $0xa,%al
   a2eae:	mov    %al,0x8(%rbx)
   a2eb1:	jmp    a2ad4 <<hyperreal::computable::node::Computable>::approx_signal+0xcc4>
   a2eb6:	movzbl 0x18(%rsp),%eax
   a2ebb:	mov    %al,0x8(%rbx)
   a2ebe:	jmp    a2ad4 <<hyperreal::computable::node::Computable>::approx_signal+0xcc4>
   a2ec3:	mov    %al,0x8(%rbx)
   a2ec6:	movq   $0xffffffffffffffff,(%rbx)
   a2ecd:	test   %r12,%r12
   a2ed0:	je     a2adb <<hyperreal::computable::node::Computable>::approx_signal+0xccb>
   a2ed6:	mov    0xb8(%rsp),%rdi
   a2ede:	shl    $0x3,%r12
   a2ee2:	mov    $0x8,%edx
   a2ee7:	mov    %r12,%rsi
   a2eea:	call   *0xe3b20(%rip)        # 186a10 <_DYNAMIC+0x240>
   a2ef0:	jmp    a2adb <<hyperreal::computable::node::Computable>::approx_signal+0xccb>
   a2ef5:	lea    0xe0e54(%rip),%rdi        # 183d50 <anon.953f3db8f5a996f677fb6a53175227a4.4.llvm.13083929455518204517>
   a2efc:	call   *0xe3c8e(%rip)        # 186b90 <_DYNAMIC+0x3c0>
   a2f02:	jmp    a2fe8 <<hyperreal::computable::node::Computable>::approx_signal+0x11d8>
   a2f07:	mov    %al,0x8(%rbx)
   a2f0a:	jmp    a2ad4 <<hyperreal::computable::node::Computable>::approx_signal+0xcc4>
   a2f0f:	mov    %al,0x8(%rbx)
   a2f12:	movq   $0xffffffffffffffff,(%rbx)
   a2f19:	test   %r12,%r12
   a2f1c:	je     a2adb <<hyperreal::computable::node::Computable>::approx_signal+0xccb>
   a2f22:	mov    0xd8(%rsp),%rdi
   a2f2a:	jmp    a2ede <<hyperreal::computable::node::Computable>::approx_signal+0x10ce>
   a2f2c:	mov    $0x20,%esi
   a2f31:	lea    -0x6e598(%rip),%rax        # 349a0 <anon.7b75c566c6a805bace2b01974d435009.54.llvm.16730574089061123067+0xac>
   a2f38:	mov    %rax,0x30(%rsp)
   a2f3d:	lea    0xdfb74(%rip),%rax        # 182ab8 <anon.7b75c566c6a805bace2b01974d435009.258.llvm.16730574089061123067+0x78>
   a2f44:	mov    %rax,0x38(%rsp)
   a2f49:	jmp    a2fa9 <<hyperreal::computable::node::Computable>::approx_signal+0x1199>
   a2f4b:	mov    $0x1b,%esi
   a2f50:	lea    -0x6da01(%rip),%rax        # 35556 <anon.7b75c566c6a805bace2b01974d435009.257.llvm.16730574089061123067+0x6b>
   a2f57:	mov    %rax,0x30(%rsp)
   a2f5c:	lea    0xdfb25(%rip),%rax        # 182a88 <anon.7b75c566c6a805bace2b01974d435009.258.llvm.16730574089061123067+0x48>
   a2f63:	mov    %rax,0x38(%rsp)
   a2f68:	jmp    a2fa9 <<hyperreal::computable::node::Computable>::approx_signal+0x1199>
   a2f6a:	mov    $0x20,%esi
   a2f6f:	jmp    a2fa9 <<hyperreal::computable::node::Computable>::approx_signal+0x1199>
   a2f71:	lea    -0x6da07(%rip),%rdi        # 35571 <anon.7b75c566c6a805bace2b01974d435009.257.llvm.16730574089061123067+0x86>
   a2f78:	lea    0xdfb21(%rip),%rdx        # 182aa0 <anon.7b75c566c6a805bace2b01974d435009.258.llvm.16730574089061123067+0x60>
   a2f7f:	mov    $0x1b,%esi
   a2f84:	call   *0xe3ce6(%rip)        # 186c70 <_DYNAMIC+0x4a0>
   a2f8a:	jmp    a2fe8 <<hyperreal::computable::node::Computable>::approx_signal+0x11d8>
   a2f8c:	lea    0xdfb3d(%rip),%rax        # 182ad0 <anon.7b75c566c6a805bace2b01974d435009.258.llvm.16730574089061123067+0x90>
   a2f93:	mov    %rax,0x38(%rsp)
   a2f98:	mov    $0x22,%esi
   a2f9d:	lea    -0x6da18(%rip),%rax        # 3558c <anon.7b75c566c6a805bace2b01974d435009.257.llvm.16730574089061123067+0xa1>
   a2fa4:	mov    %rax,0x30(%rsp)
   a2fa9:	mov    0x30(%rsp),%rdi
   a2fae:	mov    0x38(%rsp),%rdx
   a2fb3:	call   *0xe3cb7(%rip)        # 186c70 <_DYNAMIC+0x4a0>
   a2fb9:	jmp    a2fe8 <<hyperreal::computable::node::Computable>::approx_signal+0x11d8>
   a2fbb:	movb   $0xa,0x8(%rbx)
   a2fbf:	jmp    a2ad4 <<hyperreal::computable::node::Computable>::approx_signal+0xcc4>
   a2fc4:	lea    0xdfd75(%rip),%rdx        # 182d40 <anon.7b75c566c6a805bace2b01974d435009.258.llvm.16730574089061123067+0x300>
   a2fcb:	mov    $0x4,%esi
   a2fd0:	call   *0xe3a7a(%rip)        # 186a50 <_DYNAMIC+0x280>
   a2fd6:	lea    0xdfa7b(%rip),%rdx        # 182a58 <anon.7b75c566c6a805bace2b01974d435009.258.llvm.16730574089061123067+0x18>
   a2fdd:	mov    $0x10,%esi
   a2fe2:	call   *0xe3a68(%rip)        # 186a50 <_DYNAMIC+0x280>
   a2fe8:	ud2
   a2fea:	jmp    a314a <<hyperreal::computable::node::Computable>::approx_signal+0x133a>
   a2fef:	mov    %rax,%rbx
   a2ff2:	test   %r12,%r12
   a2ff5:	je     a314d <<hyperreal::computable::node::Computable>::approx_signal+0x133d>
   a2ffb:	mov    0xd8(%rsp),%rdi
   a3003:	jmp    a301e <<hyperreal::computable::node::Computable>::approx_signal+0x120e>
   a3005:	jmp    a314a <<hyperreal::computable::node::Computable>::approx_signal+0x133a>
   a300a:	mov    %rax,%rbx
   a300d:	test   %r12,%r12
   a3010:	je     a314d <<hyperreal::computable::node::Computable>::approx_signal+0x133d>
   a3016:	mov    0xb8(%rsp),%rdi
   a301e:	shl    $0x3,%r12
   a3022:	mov    $0x8,%edx
   a3027:	jmp    a313d <<hyperreal::computable::node::Computable>::approx_signal+0x132d>
   a302c:	jmp    a314a <<hyperreal::computable::node::Computable>::approx_signal+0x133a>
   a3031:	jmp    a314a <<hyperreal::computable::node::Computable>::approx_signal+0x133a>
   a3036:	jmp    a314a <<hyperreal::computable::node::Computable>::approx_signal+0x133a>
   a303b:	jmp    a3129 <<hyperreal::computable::node::Computable>::approx_signal+0x1319>
   a3040:	mov    %rax,%rbx
   a3043:	mov    0x330(%rsp),%rsi
   a304b:	test   %rsi,%rsi
   a304e:	je     a314d <<hyperreal::computable::node::Computable>::approx_signal+0x133d>
   a3054:	mov    0x338(%rsp),%rdi
   a305c:	jmp    a3097 <<hyperreal::computable::node::Computable>::approx_signal+0x1287>
   a305e:	jmp    a314a <<hyperreal::computable::node::Computable>::approx_signal+0x133a>
   a3063:	mov    %rax,%rbx
   a3066:	mov    0x70(%rsp),%rsi
   a306b:	test   %rsi,%rsi
   a306e:	je     a314d <<hyperreal::computable::node::Computable>::approx_signal+0x133d>
   a3074:	mov    0x78(%rsp),%rdi
   a3079:	jmp    a3097 <<hyperreal::computable::node::Computable>::approx_signal+0x1287>
   a307b:	mov    %rax,%rbx
   a307e:	mov    0x90(%rsp),%rsi
   a3086:	test   %rsi,%rsi
   a3089:	je     a314d <<hyperreal::computable::node::Computable>::approx_signal+0x133d>
   a308f:	mov    0x98(%rsp),%rdi
   a3097:	shl    $0x3,%rsi
   a309b:	mov    $0x8,%edx
   a30a0:	call   *0xe396a(%rip)        # 186a10 <_DYNAMIC+0x240>
   a30a6:	jmp    a314d <<hyperreal::computable::node::Computable>::approx_signal+0x133d>
   a30ab:	mov    %rax,%rbx
   a30ae:	test   %r15,%r15
   a30b1:	je     a317e <<hyperreal::computable::node::Computable>::approx_signal+0x136e>
   a30b7:	mov    0xf8(%rsp),%rdi
   a30bf:	shl    $0x3,%r15
   a30c3:	mov    $0x8,%edx
   a30c8:	mov    %r15,%rsi
   a30cb:	jmp    a3178 <<hyperreal::computable::node::Computable>::approx_signal+0x1368>
   a30d0:	mov    %rax,%rbx
   a30d3:	test   %r12,%r12
   a30d6:	je     a317e <<hyperreal::computable::node::Computable>::approx_signal+0x136e>
   a30dc:	mov    0xf8(%rsp),%rdi
   a30e4:	shl    $0x3,%r12
   a30e8:	mov    $0x8,%edx
   a30ed:	mov    %r12,%rsi
   a30f0:	jmp    a3178 <<hyperreal::computable::node::Computable>::approx_signal+0x1368>
   a30f5:	jmp    a314a <<hyperreal::computable::node::Computable>::approx_signal+0x133a>
   a30f7:	mov    %rax,%rbx
   a30fa:	mov    0x10(%rsp),%rsi
   a30ff:	test   %rsi,%rsi
   a3102:	je     a317e <<hyperreal::computable::node::Computable>::approx_signal+0x136e>
   a3104:	mov    0x18(%rsp),%rdi
   a3109:	jmp    a3123 <<hyperreal::computable::node::Computable>::approx_signal+0x1313>
   a310b:	mov    %rax,%rbx
   a310e:	mov    0xf0(%rsp),%rsi
   a3116:	test   %rsi,%rsi
   a3119:	je     a317e <<hyperreal::computable::node::Computable>::approx_signal+0x136e>
   a311b:	mov    0xf8(%rsp),%rdi
   a3123:	shl    $0x3,%rsi
   a3127:	jmp    a3173 <<hyperreal::computable::node::Computable>::approx_signal+0x1363>
   a3129:	mov    %rax,%rbx
   a312c:	test   %r12,%r12
   a312f:	je     a314d <<hyperreal::computable::node::Computable>::approx_signal+0x133d>
   a3131:	shl    $0x3,%r12
   a3135:	mov    $0x8,%edx
   a313a:	mov    %r13,%rdi
   a313d:	mov    %r12,%rsi
   a3140:	call   *0xe38ca(%rip)        # 186a10 <_DYNAMIC+0x240>
   a3146:	jmp    a314d <<hyperreal::computable::node::Computable>::approx_signal+0x133d>
   a3148:	jmp    a314a <<hyperreal::computable::node::Computable>::approx_signal+0x133a>
   a314a:	mov    %rax,%rbx
   a314d:	lea    0x210(%rsp),%rdi
   a3155:	call   7dd70 <core::ptr::drop_glue::<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>>
   a315a:	mov    0xf0(%rsp),%rsi
   a3162:	test   %rsi,%rsi
   a3165:	je     a317e <<hyperreal::computable::node::Computable>::approx_signal+0x136e>
   a3167:	mov    0xf8(%rsp),%rdi
   a316f:	shl    $0x4,%rsi
   a3173:	mov    $0x8,%edx
   a3178:	call   *0xe3892(%rip)        # 186a10 <_DYNAMIC+0x240>
   a317e:	mov    %rbx,%rdi
   a3181:	call   180f40 <_Unwind_Resume@plt>
