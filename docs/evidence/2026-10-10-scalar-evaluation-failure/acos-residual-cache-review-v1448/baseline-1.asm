
target/typed-api-prototype-v1418-acos-warm-baseline-cost/probe:     file format elf64-x86-64


Disassembly of section .text:

00000000000a1d90 <<hyperreal::computable::node::Computable>::approx_signal>:
   a1d90:	push   %rbp
   a1d91:	push   %r15
   a1d93:	push   %r14
   a1d95:	push   %r13
   a1d97:	push   %r12
   a1d99:	push   %rbx
   a1d9a:	sub    $0x3a8,%rsp
   a1da1:	mov    %ecx,%ebp
   a1da3:	mov    %rsi,%r13
   a1da6:	mov    %rdi,%rbx
   a1da9:	mov    (%rdx),%r14
   a1dac:	test   %r14,%r14
   a1daf:	je     a1dbe <<hyperreal::computable::node::Computable>::approx_signal+0x2e>
   a1db1:	movzbl 0x10(%r14),%eax
   a1db6:	test   %al,%al
   a1db8:	jne    a1f60 <<hyperreal::computable::node::Computable>::approx_signal+0x1d0>
   a1dbe:	mov    %rdx,0x40(%rsp)
   a1dc3:	data16 data16 data16 mov %fs:0x0,%rax
   a1dcf:	lea    -0x120(%rax),%r15
   a1dd6:	movzbl -0x110(%rax),%eax
   a1ddd:	test   %eax,%eax
   a1ddf:	jne    a2d60 <<hyperreal::computable::node::Computable>::approx_signal+0xfd0>
   a1de5:	movabs $0x7fffffffffffffff,%rax
   a1def:	mov    (%r15),%rcx
   a1df2:	cmp    %rax,%rcx
   a1df5:	jae    a2d90 <<hyperreal::computable::node::Computable>::approx_signal+0x1000>
   a1dfb:	lea    0x1(%rcx),%rax
   a1dff:	mov    %rax,(%r15)
   a1e02:	mov    0x8(%r15),%rdx
   a1e06:	test   %rdx,%rdx
   a1e09:	je     a1e23 <<hyperreal::computable::node::Computable>::approx_signal+0x93>
   a1e0b:	nopl   0x0(%rax,%rax,1)
   a1e10:	cmpb   $0x0,0x20(%rdx)
   a1e14:	jne    a1f5d <<hyperreal::computable::node::Computable>::approx_signal+0x1cd>
   a1e1a:	mov    0x18(%rdx),%rdx
   a1e1e:	test   %rdx,%rdx
   a1e21:	jne    a1e10 <<hyperreal::computable::node::Computable>::approx_signal+0x80>
   a1e23:	mov    %rcx,(%r15)
   a1e26:	mov    0x0(%r13),%r12
   a1e2a:	movzbl 0x10(%r12),%edx
   a1e30:	test   %edx,%edx
   a1e32:	jne    a1e53 <<hyperreal::computable::node::Computable>::approx_signal+0xc3>
   a1e34:	cmpb   $0x1,0x30(%r12)
   a1e3a:	jne    a1e53 <<hyperreal::computable::node::Computable>::approx_signal+0xc3>
   a1e3c:	movups -0x6d3c3(%rip),%xmm0        # 34a80 <anon.7b75c566c6a805bace2b01974d435009.137.llvm.17861709745643738061+0x10>
   a1e43:	movups %xmm0,0x10(%rbx)
   a1e47:	movups -0x6d3de(%rip),%xmm0        # 34a70 <anon.7b75c566c6a805bace2b01974d435009.137.llvm.17861709745643738061>
   a1e4e:	jmp    a1ffe <<hyperreal::computable::node::Computable>::approx_signal+0x26e>
   a1e53:	cmp    $0xfff00000,%ebp
   a1e59:	jge    a1e6c <<hyperreal::computable::node::Computable>::approx_signal+0xdc>
   a1e5b:	neg    %ebp
   a1e5d:	setno  %al
   a1e60:	add    %al,%al
   a1e62:	add    $0xa,%al
   a1e64:	mov    %al,0x8(%rbx)
   a1e67:	jmp    a1f64 <<hyperreal::computable::node::Computable>::approx_signal+0x1d4>
   a1e6c:	cmp    $0x2,%edx
   a1e6f:	je     a1f80 <<hyperreal::computable::node::Computable>::approx_signal+0x1f0>
   a1e75:	cmp    $0x1,%edx
   a1e78:	je     a1edf <<hyperreal::computable::node::Computable>::approx_signal+0x14f>
   a1e7a:	test   %edx,%edx
   a1e7c:	jne    a1faa <<hyperreal::computable::node::Computable>::approx_signal+0x21a>
   a1e82:	mov    %rax,(%r15)
   a1e85:	mov    0x8(%r15),%rdx
   a1e89:	test   %rdx,%rdx
   a1e8c:	je     a1eba <<hyperreal::computable::node::Computable>::approx_signal+0x12a>
   a1e8e:	mov    %rdx,%rsi
   a1e91:	cmpb   $0x0,0x20(%rsi)
   a1e95:	jne    a1ed0 <<hyperreal::computable::node::Computable>::approx_signal+0x140>
   a1e97:	mov    0x10(%rsi),%rdi
   a1e9b:	mov    0x18(%rdi),%rax
   a1e9f:	nop
   a1ea0:	test   %rax,%rax
   a1ea3:	je     a1ed0 <<hyperreal::computable::node::Computable>::approx_signal+0x140>
   a1ea5:	lea    -0x1(%rax),%r8
   a1ea9:	lock cmpxchg %r8,0x18(%rdi)
   a1eaf:	jne    a1ea0 <<hyperreal::computable::node::Computable>::approx_signal+0x110>
   a1eb1:	mov    0x18(%rsi),%rsi
   a1eb5:	test   %rsi,%rsi
   a1eb8:	jne    a1e91 <<hyperreal::computable::node::Computable>::approx_signal+0x101>
   a1eba:	mov    %rcx,(%r15)
   a1ebd:	add    $0x18,%r12
   a1ec1:	mov    %rbx,%rdi
   a1ec4:	mov    %r12,%rsi
   a1ec7:	jmp    a1f39 <<hyperreal::computable::node::Computable>::approx_signal+0x1a9>
   a1ec9:	nopl   0x0(%rax)
   a1ed0:	movb   $0x1,0x20(%rdx)
   a1ed4:	mov    0x18(%rdx),%rdx
   a1ed8:	test   %rdx,%rdx
   a1edb:	jne    a1ed0 <<hyperreal::computable::node::Computable>::approx_signal+0x140>
   a1edd:	jmp    a1f5d <<hyperreal::computable::node::Computable>::approx_signal+0x1cd>
   a1edf:	mov    %rax,(%r15)
   a1ee2:	mov    0x8(%r15),%rdx
   a1ee6:	test   %rdx,%rdx
   a1ee9:	je     a1f1a <<hyperreal::computable::node::Computable>::approx_signal+0x18a>
   a1eeb:	mov    %rdx,%rsi
   a1eee:	cmpb   $0x0,0x20(%rsi)
   a1ef2:	jne    a1f50 <<hyperreal::computable::node::Computable>::approx_signal+0x1c0>
   a1ef4:	mov    0x10(%rsi),%rdi
   a1ef8:	mov    0x18(%rdi),%rax
   a1efc:	nopl   0x0(%rax)
   a1f00:	test   %rax,%rax
   a1f03:	je     a1f50 <<hyperreal::computable::node::Computable>::approx_signal+0x1c0>
   a1f05:	lea    -0x1(%rax),%r8
   a1f09:	lock cmpxchg %r8,0x18(%rdi)
   a1f0f:	jne    a1f00 <<hyperreal::computable::node::Computable>::approx_signal+0x170>
   a1f11:	mov    0x18(%rsi),%rsi
   a1f15:	test   %rsi,%rsi
   a1f18:	jne    a1eee <<hyperreal::computable::node::Computable>::approx_signal+0x15e>
   a1f1a:	mov    %rcx,(%r15)
   a1f1d:	lea    0x115cdc(%rip),%rdi        # 1b7c00 <hyperreal::computable::node::signed::ONE+0x20>
   a1f24:	lea    0x115cb5(%rip),%r14        # 1b7be0 <hyperreal::computable::node::signed::ONE>
   a1f2b:	mov    %r14,%rsi
   a1f2e:	call   60c20 <<std::sync::once::Once>::call_once_force::<<std::sync::lazy_lock::LazyLock<num_bigint::bigint::BigInt>>::force::{closure#0}>>
   a1f33:	mov    %rbx,%rdi
   a1f36:	mov    %r14,%rsi
   a1f39:	mov    %ebp,%edx
   a1f3b:	call   bc570 <hyperreal::computable::node::scale_at_precision_ref>
   a1f40:	jmp    a1f6b <<hyperreal::computable::node::Computable>::approx_signal+0x1db>
   a1f42:	data16 data16 data16 data16 cs nopw 0x0(%rax,%rax,1)
   a1f50:	movb   $0x1,0x20(%rdx)
   a1f54:	mov    0x18(%rdx),%rdx
   a1f58:	test   %rdx,%rdx
   a1f5b:	jne    a1f50 <<hyperreal::computable::node::Computable>::approx_signal+0x1c0>
   a1f5d:	mov    %rcx,(%r15)
   a1f60:	movb   $0xc,0x8(%rbx)
   a1f64:	movq   $0xffffffffffffffff,(%rbx)
   a1f6b:	mov    %rbx,%rax
   a1f6e:	add    $0x3a8,%rsp
   a1f75:	pop    %rbx
   a1f76:	pop    %r12
   a1f78:	pop    %r13
   a1f7a:	pop    %r14
   a1f7c:	pop    %r15
   a1f7e:	pop    %rbp
   a1f7f:	ret
   a1f80:	movzbl 0x11(%r12),%esi
   a1f86:	lea    0x210(%rsp),%rdi
   a1f8e:	mov    %ebp,%edx
   a1f90:	call   b0670 <<hyperreal::computable::node::Computable>::cached_constant_at_precision>
   a1f95:	cmpq   $0xffffffffffffffff,0x210(%rsp)
   a1f9e:	jne    a1fea <<hyperreal::computable::node::Computable>::approx_signal+0x25a>
   a1fa0:	movl   $0x0,0xc(%rsp)
   a1fa8:	jmp    a200c <<hyperreal::computable::node::Computable>::approx_signal+0x27c>
   a1faa:	mov    0x38(%r12),%rax
   a1faf:	test   $0x800,%eax
   a1fb4:	jne    a1fcb <<hyperreal::computable::node::Computable>::approx_signal+0x23b>
   a1fb6:	mov    %r13,%rdi
   a1fb9:	mov    0x40(%rsp),%rsi
   a1fbe:	call   b77b0 <<hyperreal::computable::node::Computable>::ensure_finite_real>
   a1fc3:	cmp    $0xff,%al
   a1fc5:	jne    a1e64 <<hyperreal::computable::node::Computable>::approx_signal+0xd4>
   a1fcb:	lea    0x40(%r12),%rsi
   a1fd0:	lea    0x210(%rsp),%rdi
   a1fd8:	mov    %ebp,%edx
   a1fda:	call   bade0 <<hyperreal::computable::node::EvaluationCache>::at_precision>
   a1fdf:	cmpq   $0xffffffffffffffff,0x210(%rsp)
   a1fe8:	je     a2006 <<hyperreal::computable::node::Computable>::approx_signal+0x276>
   a1fea:	movups 0x210(%rsp),%xmm0
   a1ff2:	movups 0x220(%rsp),%xmm1
   a1ffa:	movups %xmm1,0x10(%rbx)
   a1ffe:	movups %xmm0,(%rbx)
   a2001:	jmp    a1f6b <<hyperreal::computable::node::Computable>::approx_signal+0x1db>
   a2006:	mov    $0x1,%al
   a2008:	mov    %eax,0xc(%rsp)
   a200c:	lea    0x10(%r12),%rax
   a2011:	mov    %rax,0x38(%rsp)
   a2016:	movzbl (%rax),%eax
   a2019:	add    $0xffffffdc,%eax
   a201c:	cmp    $0x1a,%eax
   a201f:	ja     a20bb <<hyperreal::computable::node::Computable>::approx_signal+0x32b>
   a2025:	lea    -0x6e5f8(%rip),%rcx        # 33a34 <anon.7b75c566c6a805bace2b01974d435009.371.llvm.17861709745643738061+0xb18>
   a202c:	movslq (%rcx,%rax,4),%rax
   a2030:	add    %rcx,%rax
   a2033:	jmp    *%rax
   a2035:	movl   $0x1,0x30(%rsp)
   a203d:	jmp    a205b <<hyperreal::computable::node::Computable>::approx_signal+0x2cb>
   a203f:	movl   $0x0,0x30(%rsp)
   a2047:	jmp    a205b <<hyperreal::computable::node::Computable>::approx_signal+0x2cb>
   a2049:	movl   $0x2,0x30(%rsp)
   a2051:	jmp    a205b <<hyperreal::computable::node::Computable>::approx_signal+0x2cb>
   a2053:	movl   $0x3,0x30(%rsp)
   a205b:	lea    0x18(%r12),%rdi
   a2060:	call   b7330 <<hyperreal::computable::node::Computable>::half_pi_index_when_real>
   a2065:	test   $0x1,%al
   a2067:	je     a20bb <<hyperreal::computable::node::Computable>::approx_signal+0x32b>
   a2069:	movzbl %dl,%edi
   a206c:	movl   $0xff000100,0x210(%rsp)
   a2077:	cmp    $0x3,%dil
   a207b:	ja     a2f44 <<hyperreal::computable::node::Computable>::approx_signal+0x11b4>
   a2081:	movzbl 0x210(%rsp,%rdi,1),%ecx
   a2089:	movl   $0xff0001,0x210(%rsp)
   a2094:	movzbl 0x210(%rsp,%rdi,1),%eax
   a209c:	mov    0x30(%rsp),%edx
   a20a0:	lea    -0x6e607(%rip),%rsi        # 33aa0 <anon.7b75c566c6a805bace2b01974d435009.371.llvm.17861709745643738061+0xb84>
   a20a7:	movslq (%rsi,%rdx,4),%rdx
   a20ab:	add    %rsi,%rdx
   a20ae:	jmp    *%rdx
   a20b0:	mov    %ecx,%eax
   a20b2:	movsbq %al,%rsi
   a20b6:	jmp    a2a94 <<hyperreal::computable::node::Computable>::approx_signal+0xd04>
   a20bb:	cmpb   $0x0,0xc(%rsp)
   a20c0:	je     a20f5 <<hyperreal::computable::node::Computable>::approx_signal+0x365>
   a20c2:	lea    0x210(%rsp),%rdi
   a20ca:	mov    %r13,%rsi
   a20cd:	mov    0x40(%rsp),%rdx
   a20d2:	mov    %ebp,%ecx
   a20d4:	call   a10b0 <<hyperreal::computable::node::Computable>::cached_quadratic_tower_approximation>
   a20d9:	mov    0x210(%rsp),%r12
   a20e1:	cmp    $0xffffffffffffffff,%r12
   a20e5:	je     a2b62 <<hyperreal::computable::node::Computable>::approx_signal+0xdd2>
   a20eb:	cmp    $0xfffffffffffffffe,%r12
   a20ef:	jne    a2c58 <<hyperreal::computable::node::Computable>::approx_signal+0xec8>
   a20f5:	mov    0x38(%rsp),%rsi
   a20fa:	movzbl (%rsi),%eax
   a20fd:	cmp    $0xa,%eax
   a2100:	ja     a2b6f <<hyperreal::computable::node::Computable>::approx_signal+0xddf>
   a2106:	mov    $0x430,%ecx
   a210b:	bt     %eax,%ecx
   a210e:	jae    a2b6f <<hyperreal::computable::node::Computable>::approx_signal+0xddf>
   a2114:	movl   $0xffffffff,0x108(%rsp)
   a211f:	movl   $0xffffffff,0x118(%rsp)
   a212a:	movl   $0xffffffff,0x128(%rsp)
   a2135:	movl   $0xffffffff,0x138(%rsp)
   a2140:	movl   $0xffffffff,0x148(%rsp)
   a214b:	movl   $0xffffffff,0x158(%rsp)
   a2156:	movl   $0xffffffff,0x168(%rsp)
   a2161:	movl   $0xffffffff,0x178(%rsp)
   a216c:	movl   $0xffffffff,0x188(%rsp)
   a2177:	movl   $0xffffffff,0x198(%rsp)
   a2182:	movl   $0xffffffff,0x1a8(%rsp)
   a218d:	movl   $0xffffffff,0x1b8(%rsp)
   a2198:	movl   $0xffffffff,0x1c8(%rsp)
   a21a3:	movl   $0xffffffff,0x1d8(%rsp)
   a21ae:	movl   $0xffffffff,0x1e8(%rsp)
   a21b9:	movl   $0xffffffff,0x1f8(%rsp)
   a21c4:	movq   $0x0,0x208(%rsp)
   a21d0:	movq   $0x0,0xf0(%rsp)
   a21dc:	movq   $0x8,0xf8(%rsp)
   a21e8:	movq   $0x0,0x100(%rsp)
   a21f4:	movq   $0xffffffffffffffff,0x228(%rsp)
   a2200:	movq   $0xffffffffffffffff,0x248(%rsp)
   a220c:	movq   $0xffffffffffffffff,0x268(%rsp)
   a2218:	movq   $0xffffffffffffffff,0x288(%rsp)
   a2224:	movq   $0xffffffffffffffff,0x2a8(%rsp)
   a2230:	movq   $0xffffffffffffffff,0x2c8(%rsp)
   a223c:	movq   $0xffffffffffffffff,0x2e8(%rsp)
   a2248:	movq   $0xffffffffffffffff,0x308(%rsp)
   a2254:	movq   $0x0,0x328(%rsp)
   a2260:	movq   $0x0,0x210(%rsp)
   a226c:	movq   $0x8,0x218(%rsp)
   a2278:	movq   $0x0,0x220(%rsp)
   a2284:	mov    %r13,0x18(%rsp)
   a2289:	mov    %ebp,0x14(%rsp)
   a228d:	movl   $0x0,0x10(%rsp)
   a2295:	lea    0xf0(%rsp),%rdi
   a229d:	lea    0x10(%rsp),%rsi
   a22a2:	call   a1790 <<hyperreal::computable::node::InlineStack<<hyperreal::computable::node::Computable>::approx_signal::Frame, 16>>::push>
   a22a7:	lea    0xe0392(%rip),%rax        # 182640 <anon.7b75c566c6a805bace2b01974d435009.258.llvm.17861709745643738061+0x30>
   a22ae:	mov    %rax,0x38(%rsp)
   a22b3:	lea    -0x6d9aa(%rip),%rax        # 34910 <anon.7b75c566c6a805bace2b01974d435009.54.llvm.17861709745643738061+0x6c>
   a22ba:	mov    %rax,0x30(%rsp)
   a22bf:	mov    0x100(%rsp),%rax
   a22c7:	test   %rax,%rax
   a22ca:	je     a22f3 <<hyperreal::computable::node::Computable>::approx_signal+0x563>
   a22cc:	dec    %rax
   a22cf:	mov    %rax,0x100(%rsp)
   a22d7:	mov    0xf8(%rsp),%rcx
   a22df:	shl    $0x4,%rax
   a22e3:	mov    (%rcx,%rax,1),%r12d
   a22e7:	mov    0x4(%rcx,%rax,1),%r9d
   a22ec:	mov    0x8(%rcx,%rax,1),%r13
   a22f1:	jmp    a233b <<hyperreal::computable::node::Computable>::approx_signal+0x5ab>
   a22f3:	mov    0x208(%rsp),%rax
   a22fb:	test   %rax,%rax
   a22fe:	je     a2d2c <<hyperreal::computable::node::Computable>::approx_signal+0xf9c>
   a2304:	lea    -0x1(%rax),%rdi
   a2308:	mov    %rdi,0x208(%rsp)
   a2310:	cmp    $0x10,%rax
   a2314:	ja     a2f56 <<hyperreal::computable::node::Computable>::approx_signal+0x11c6>
   a231a:	shl    $0x4,%rdi
   a231e:	lea    0x108(%rsp),%rax
   a2326:	mov    (%rax,%rdi,1),%r12d
   a232a:	mov    0x4(%rax,%rdi,1),%r9d
   a232f:	mov    0x8(%rax,%rdi,1),%r13
   a2334:	movl   $0xffffffff,(%rax,%rdi,1)
   a233b:	cmp    $0xffffffff,%r12d
   a233f:	je     a2d2c <<hyperreal::computable::node::Computable>::approx_signal+0xf9c>
   a2345:	test   %r14,%r14
   a2348:	je     a2357 <<hyperreal::computable::node::Computable>::approx_signal+0x5c7>
   a234a:	movzbl 0x10(%r14),%eax
   a234f:	test   %al,%al
   a2351:	jne    a2a50 <<hyperreal::computable::node::Computable>::approx_signal+0xcc0>
   a2357:	movzbl 0x10(%r15),%eax
   a235c:	test   %eax,%eax
   a235e:	jne    a29e6 <<hyperreal::computable::node::Computable>::approx_signal+0xc56>
   a2364:	mov    (%r15),%rcx
   a2367:	movabs $0x7fffffffffffffff,%rax
   a2371:	cmp    %rax,%rcx
   a2374:	jae    a2a20 <<hyperreal::computable::node::Computable>::approx_signal+0xc90>
   a237a:	lea    0x1(%rcx),%rax
   a237e:	mov    %rax,(%r15)
   a2381:	mov    0x8(%r15),%rdx
   a2385:	test   %rdx,%rdx
   a2388:	je     a23be <<hyperreal::computable::node::Computable>::approx_signal+0x62e>
   a238a:	mov    %rdx,%rsi
   a238d:	cmpb   $0x0,0x20(%rsi)
   a2391:	jne    a2a40 <<hyperreal::computable::node::Computable>::approx_signal+0xcb0>
   a2397:	mov    0x10(%rsi),%rdi
   a239b:	mov    0x18(%rdi),%rax
   a239f:	nop
   a23a0:	test   %rax,%rax
   a23a3:	je     a2a40 <<hyperreal::computable::node::Computable>::approx_signal+0xcb0>
   a23a9:	lea    -0x1(%rax),%r8
   a23ad:	lock cmpxchg %r8,0x18(%rdi)
   a23b3:	jne    a23a0 <<hyperreal::computable::node::Computable>::approx_signal+0x610>
   a23b5:	mov    0x18(%rsi),%rsi
   a23b9:	test   %rsi,%rsi
   a23bc:	jne    a238d <<hyperreal::computable::node::Computable>::approx_signal+0x5fd>
   a23be:	mov    %r9d,0xc(%rsp)
   a23c3:	mov    %rcx,(%r15)
   a23c6:	mov    %r12d,%eax
   a23c9:	lea    -0x6e920(%rip),%rcx        # 33ab0 <anon.7b75c566c6a805bace2b01974d435009.371.llvm.17861709745643738061+0xb94>
   a23d0:	movslq (%rcx,%rax,4),%rax
   a23d4:	add    %rcx,%rax
   a23d7:	jmp    *%rax
   a23d9:	mov    0x0(%r13),%rsi
   a23dd:	cmpb   $0x0,0x10(%rsi)
   a23e1:	jne    a23f0 <<hyperreal::computable::node::Computable>::approx_signal+0x660>
   a23e3:	cmpb   $0x1,0x30(%rsi)
   a23e7:	lea    -0x6d97e(%rip),%rax        # 34a70 <anon.7b75c566c6a805bace2b01974d435009.137.llvm.17861709745643738061>
   a23ee:	je     a2424 <<hyperreal::computable::node::Computable>::approx_signal+0x694>
   a23f0:	mov    0xc(%rsp),%edx
   a23f4:	cmp    $0xffefffff,%edx
   a23fa:	jle    a2e25 <<hyperreal::computable::node::Computable>::approx_signal+0x1095>
   a2400:	lea    0x388(%rsp),%rdi
   a2408:	call   ae090 <<hyperreal::computable::node::Computable>::cached_at_precision>
   a240d:	cmpq   $0xffffffffffffffff,0x388(%rsp)
   a2416:	lea    0x388(%rsp),%rax
   a241e:	je     a26b3 <<hyperreal::computable::node::Computable>::approx_signal+0x923>
   a2424:	lea    0x210(%rsp),%rdi
   a242c:	mov    %rax,%rsi
   a242f:	call   a16b0 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::push>
   a2434:	jmp    a22bf <<hyperreal::computable::node::Computable>::approx_signal+0x52f>
   a2439:	lea    0x10(%rsp),%rdi
   a243e:	lea    0x210(%rsp),%rsi
   a2446:	call   a1620 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::pop>
   a244b:	mov    %r13,0x68(%rsp)
   a2450:	mov    0x10(%rsp),%r12
   a2455:	cmp    $0xffffffffffffffff,%r12
   a2459:	je     a2ecb <<hyperreal::computable::node::Computable>::approx_signal+0x113b>
   a245f:	mov    0x18(%rsp),%r13
   a2464:	lea    0x29(%rsp),%rax
   a2469:	movups -0x9(%rax),%xmm0
   a246d:	movaps %xmm0,0x350(%rsp)
   a2475:	lea    0x10(%rsp),%rdi
   a247a:	lea    0x210(%rsp),%rsi
   a2482:	call   a1620 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::pop>
   a2487:	cmpq   $0xffffffffffffffff,0x10(%rsp)
   a248d:	je     a2ef1 <<hyperreal::computable::node::Computable>::approx_signal+0x1161>
   a2493:	movups 0x10(%rsp),%xmm0
   a2498:	movups 0x20(%rsp),%xmm1
   a249d:	movaps %xmm1,0x370(%rsp)
   a24a5:	movaps %xmm0,0x360(%rsp)
   a24ad:	mov    %r12,0x10(%rsp)
   a24b2:	mov    %r13,0x18(%rsp)
   a24b7:	movaps 0x350(%rsp),%xmm0
   a24bf:	lea    0x29(%rsp),%rax
   a24c4:	movups %xmm0,-0x9(%rax)
   a24c8:	lea    0x48(%rsp),%rbp
   a24cd:	mov    %rbp,%rdi
   a24d0:	lea    0x360(%rsp),%rsi
   a24d8:	lea    0x10(%rsp),%rdx
   a24dd:	call   c0340 <<num_bigint::bigint::BigInt as core::ops::arith::Add>::add>
   a24e2:	lea    0x330(%rsp),%r13
   a24ea:	mov    %r13,%rdi
   a24ed:	mov    %rbp,%rsi
   a24f0:	mov    $0xfffffffe,%edx
   a24f5:	call   bca50 <hyperreal::computable::node::scale>
   a24fa:	movzbl 0x348(%rsp),%r12d
   a2503:	mov    %rbp,%rdi
   a2506:	mov    %r13,%rsi
   a2509:	call   129370 <<alloc::vec::Vec<u64> as core::clone::Clone>::clone>
   a250e:	mov    0x58(%rsp),%rax
   a2513:	mov    %rax,0x20(%rsp)
   a2518:	movups 0x48(%rsp),%xmm0
   a251d:	movaps %xmm0,0x10(%rsp)
   a2522:	mov    %r12b,0x28(%rsp)
   a2527:	mov    0x68(%rsp),%rdi
   a252c:	mov    %r14,%rsi
   a252f:	mov    0xc(%rsp),%edx
   a2533:	lea    0x10(%rsp),%rcx
   a2538:	call   ada20 <<hyperreal::computable::node::Computable>::store_cache_value>
   a253d:	cmp    $0xff,%al
   a253f:	jne    a2dee <<hyperreal::computable::node::Computable>::approx_signal+0x105e>
   a2545:	lea    0x210(%rsp),%rdi
   a254d:	mov    %r13,%rsi
   a2550:	call   a16b0 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::push>
   a2555:	jmp    a22bf <<hyperreal::computable::node::Computable>::approx_signal+0x52f>
   a255a:	lea    0x10(%rsp),%rdi
   a255f:	lea    0x210(%rsp),%rsi
   a2567:	call   a1620 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::pop>
   a256c:	cmpq   $0xffffffffffffffff,0x10(%rsp)
   a2572:	je     a2eac <<hyperreal::computable::node::Computable>::approx_signal+0x111c>
   a2578:	movups 0x10(%rsp),%xmm0
   a257d:	movups 0x20(%rsp),%xmm1
   a2582:	movaps %xmm1,0xa0(%rsp)
   a258a:	movaps %xmm0,0x90(%rsp)
   a2592:	movzbl 0xa8(%rsp),%r12d
   a259b:	lea    0x48(%rsp),%rdi
   a25a0:	lea    0x90(%rsp),%rsi
   a25a8:	call   129370 <<alloc::vec::Vec<u64> as core::clone::Clone>::clone>
   a25ad:	mov    0xc(%rsp),%edx
   a25b1:	mov    0x58(%rsp),%rax
   a25b6:	mov    %rax,0x20(%rsp)
   a25bb:	movups 0x48(%rsp),%xmm0
   a25c0:	movaps %xmm0,0x10(%rsp)
   a25c5:	mov    %r12b,0x28(%rsp)
   a25ca:	mov    %r13,%rdi
   a25cd:	mov    %r14,%rsi
   a25d0:	lea    0x10(%rsp),%rcx
   a25d5:	call   ada20 <<hyperreal::computable::node::Computable>::store_cache_value>
   a25da:	cmp    $0xff,%al
   a25dc:	jne    a2daa <<hyperreal::computable::node::Computable>::approx_signal+0x101a>
   a25e2:	lea    0x210(%rsp),%rdi
   a25ea:	lea    0x90(%rsp),%rsi
   a25f2:	call   a16b0 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::push>
   a25f7:	jmp    a22bf <<hyperreal::computable::node::Computable>::approx_signal+0x52f>
   a25fc:	lea    0x10(%rsp),%rdi
   a2601:	lea    0x210(%rsp),%rsi
   a2609:	call   a1620 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::pop>
   a260e:	cmpq   $0xffffffffffffffff,0x10(%rsp)
   a2614:	je     a2eea <<hyperreal::computable::node::Computable>::approx_signal+0x115a>
   a261a:	mov    0x20(%rsp),%rax
   a261f:	mov    %rax,0x80(%rsp)
   a2627:	movups 0x10(%rsp),%xmm0
   a262c:	movaps %xmm0,0x70(%rsp)
   a2631:	lea    0x29(%rsp),%rcx
   a2636:	mov    (%rcx),%eax
   a2638:	mov    $0x2,%r12b
   a263b:	sub    0x28(%rsp),%r12b
   a2640:	mov    0x3(%rcx),%ecx
   a2643:	lea    0x89(%rsp),%rdx
   a264b:	mov    %ecx,0x3(%rdx)
   a264e:	mov    %eax,(%rdx)
   a2650:	mov    %r12b,0x88(%rsp)
   a2658:	lea    0x48(%rsp),%rdi
   a265d:	lea    0x70(%rsp),%rsi
   a2662:	call   129370 <<alloc::vec::Vec<u64> as core::clone::Clone>::clone>
   a2667:	mov    0xc(%rsp),%edx
   a266b:	mov    0x58(%rsp),%rax
   a2670:	mov    %rax,0x20(%rsp)
   a2675:	movups 0x48(%rsp),%xmm0
   a267a:	movaps %xmm0,0x10(%rsp)
   a267f:	mov    %r12b,0x28(%rsp)
   a2684:	mov    %r13,%rdi
   a2687:	mov    %r14,%rsi
   a268a:	lea    0x10(%rsp),%rcx
   a268f:	call   ada20 <<hyperreal::computable::node::Computable>::store_cache_value>
   a2694:	cmp    $0xff,%al
   a2696:	jne    a2dcf <<hyperreal::computable::node::Computable>::approx_signal+0x103f>
   a269c:	lea    0x210(%rsp),%rdi
   a26a4:	lea    0x70(%rsp),%rsi
   a26a9:	call   a16b0 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::push>
   a26ae:	jmp    a22bf <<hyperreal::computable::node::Computable>::approx_signal+0x52f>
   a26b3:	lea    0x10(%rsp),%rdi
   a26b8:	mov    %r13,%rsi
   a26bb:	mov    0x40(%rsp),%rdx
   a26c0:	mov    0xc(%rsp),%ecx
   a26c4:	call   a10b0 <<hyperreal::computable::node::Computable>::cached_quadratic_tower_approximation>
   a26c9:	mov    0x10(%rsp),%r12
   a26ce:	cmp    $0xfffffffffffffffe,%r12
   a26d2:	je     a277f <<hyperreal::computable::node::Computable>::approx_signal+0x9ef>
   a26d8:	cmp    $0xffffffffffffffff,%r12
   a26dc:	je     a2e36 <<hyperreal::computable::node::Computable>::approx_signal+0x10a6>
   a26e2:	mov    %r13,%rbp
   a26e5:	movzbl 0x18(%rsp),%eax
   a26ea:	lea    0x29(%rsp),%rdx
   a26ef:	mov    -0x1(%rdx),%rcx
   a26f3:	lea    0xb9(%rsp),%rsi
   a26fb:	mov    %rcx,0xf(%rsi)
   a26ff:	movups -0x10(%rdx),%xmm0
   a2703:	movups %xmm0,(%rsi)
   a2706:	mov    %r12,0xb0(%rsp)
   a270e:	mov    %al,0xb8(%rsp)
   a2715:	movzbl 0xc8(%rsp),%r13d
   a271e:	lea    0x48(%rsp),%rdi
   a2723:	lea    0xb0(%rsp),%rsi
   a272b:	call   129370 <<alloc::vec::Vec<u64> as core::clone::Clone>::clone>
   a2730:	mov    0x58(%rsp),%rax
   a2735:	mov    %rax,0x20(%rsp)
   a273a:	movups 0x48(%rsp),%xmm0
   a273f:	movaps %xmm0,0x10(%rsp)
   a2744:	mov    %r13b,0x28(%rsp)
   a2749:	mov    %rbp,%rdi
   a274c:	mov    %r14,%rsi
   a274f:	mov    0xc(%rsp),%edx
   a2753:	lea    0x10(%rsp),%rcx
   a2758:	call   ada20 <<hyperreal::computable::node::Computable>::store_cache_value>
   a275d:	cmp    $0xff,%al
   a275f:	jne    a2e43 <<hyperreal::computable::node::Computable>::approx_signal+0x10b3>
   a2765:	lea    0x210(%rsp),%rdi
   a276d:	lea    0xb0(%rsp),%rsi
   a2775:	call   a16b0 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::push>
   a277a:	jmp    a22bf <<hyperreal::computable::node::Computable>::approx_signal+0x52f>
   a277f:	mov    0x0(%r13),%r12
   a2783:	movzbl 0x10(%r12),%eax
   a2789:	cmp    $0xa,%eax
   a278c:	je     a28b6 <<hyperreal::computable::node::Computable>::approx_signal+0xb26>
   a2792:	cmp    $0x5,%eax
   a2795:	je     a27f7 <<hyperreal::computable::node::Computable>::approx_signal+0xa67>
   a2797:	cmp    $0x4,%eax
   a279a:	jne    a2920 <<hyperreal::computable::node::Computable>::approx_signal+0xb90>
   a27a0:	mov    %r13,0x18(%rsp)
   a27a5:	mov    0xc(%rsp),%eax
   a27a9:	mov    %eax,0x14(%rsp)
   a27ad:	movl   $0x1,0x10(%rsp)
   a27b5:	lea    0xf0(%rsp),%rdi
   a27bd:	lea    0x10(%rsp),%rsi
   a27c2:	call   a1790 <<hyperreal::computable::node::InlineStack<<hyperreal::computable::node::Computable>::approx_signal::Frame, 16>>::push>
   a27c7:	add    $0x18,%r12
   a27cb:	mov    %r12,0x18(%rsp)
   a27d0:	mov    0xc(%rsp),%eax
   a27d4:	mov    %eax,0x14(%rsp)
   a27d8:	movl   $0x0,0x10(%rsp)
   a27e0:	lea    0xf0(%rsp),%rdi
   a27e8:	lea    0x10(%rsp),%rsi
   a27ed:	call   a1790 <<hyperreal::computable::node::InlineStack<<hyperreal::computable::node::Computable>::approx_signal::Frame, 16>>::push>
   a27f2:	jmp    a22bf <<hyperreal::computable::node::Computable>::approx_signal+0x52f>
   a27f7:	mov    %r13,0x18(%rsp)
   a27fc:	mov    0xc(%rsp),%eax
   a2800:	mov    %eax,0x14(%rsp)
   a2804:	movl   $0x2,0x10(%rsp)
   a280c:	lea    0xf0(%rsp),%rdi
   a2814:	lea    0x10(%rsp),%rsi
   a2819:	call   a1790 <<hyperreal::computable::node::InlineStack<<hyperreal::computable::node::Computable>::approx_signal::Frame, 16>>::push>
   a281e:	lea    0x28(%r12),%rdx
   a2823:	lea    0x18(%r12),%rbp
   a2828:	mov    0x28(%r12),%rax
   a282d:	mov    0x38(%rax),%r13
   a2831:	shr    $0x10,%r13d
   a2835:	mov    0x18(%r12),%rax
   a283a:	mov    0x38(%rax),%r12
   a283e:	shr    $0x10,%r12d
   a2842:	mov    0xc(%rsp),%ecx
   a2846:	add    $0xfffffffe,%ecx
   a2849:	cmp    %r12w,%r13w
   a284d:	mov    %rdx,0x68(%rsp)
   a2852:	mov    %rdx,%rax
   a2855:	cmovg  %rbp,%rax
   a2859:	mov    %rax,0x18(%rsp)
   a285e:	mov    %ecx,0xc(%rsp)
   a2862:	mov    %ecx,0x14(%rsp)
   a2866:	movl   $0x0,0x10(%rsp)
   a286e:	lea    0xf0(%rsp),%rdi
   a2876:	lea    0x10(%rsp),%rsi
   a287b:	call   a1790 <<hyperreal::computable::node::InlineStack<<hyperreal::computable::node::Computable>::approx_signal::Frame, 16>>::push>
   a2880:	cmp    %r12w,%r13w
   a2884:	cmovg  0x68(%rsp),%rbp
   a288a:	mov    %rbp,0x18(%rsp)
   a288f:	mov    0xc(%rsp),%eax
   a2893:	mov    %eax,0x14(%rsp)
   a2897:	movl   $0x0,0x10(%rsp)
   a289f:	lea    0xf0(%rsp),%rdi
   a28a7:	lea    0x10(%rsp),%rsi
   a28ac:	call   a1790 <<hyperreal::computable::node::InlineStack<<hyperreal::computable::node::Computable>::approx_signal::Frame, 16>>::push>
   a28b1:	jmp    a22bf <<hyperreal::computable::node::Computable>::approx_signal+0x52f>
   a28b6:	mov    %r13,0x18(%rsp)
   a28bb:	mov    0xc(%rsp),%eax
   a28bf:	mov    %eax,0x14(%rsp)
   a28c3:	movl   $0x3,0x10(%rsp)
   a28cb:	lea    0xf0(%rsp),%rdi
   a28d3:	lea    0x10(%rsp),%rsi
   a28d8:	call   a1790 <<hyperreal::computable::node::InlineStack<<hyperreal::computable::node::Computable>::approx_signal::Frame, 16>>::push>
   a28dd:	mov    0xc(%rsp),%eax
   a28e1:	sub    0x14(%r12),%eax
   a28e6:	mov    %eax,0xc(%rsp)
   a28ea:	jo     a2f3b <<hyperreal::computable::node::Computable>::approx_signal+0x11ab>
   a28f0:	add    $0x18,%r12
   a28f4:	mov    %r12,0x18(%rsp)
   a28f9:	mov    0xc(%rsp),%eax
   a28fd:	mov    %eax,0x14(%rsp)
   a2901:	movl   $0x0,0x10(%rsp)
   a2909:	lea    0xf0(%rsp),%rdi
   a2911:	lea    0x10(%rsp),%rsi
   a2916:	call   a1790 <<hyperreal::computable::node::InlineStack<<hyperreal::computable::node::Computable>::approx_signal::Frame, 16>>::push>
   a291b:	jmp    a22bf <<hyperreal::computable::node::Computable>::approx_signal+0x52f>
   a2920:	add    $0x10,%r12
   a2924:	lea    0x10(%rsp),%rdi
   a2929:	mov    %r12,%rsi
   a292c:	mov    0x40(%rsp),%rdx
   a2931:	mov    0xc(%rsp),%ecx
   a2935:	call   ea400 <<hyperreal::computable::approximation::Approximation>::approximate>
   a293a:	mov    0x10(%rsp),%r12
   a293f:	movzbl 0x18(%rsp),%eax
   a2944:	cmp    $0xffffffffffffffff,%r12
   a2948:	je     a2e87 <<hyperreal::computable::node::Computable>::approx_signal+0x10f7>
   a294e:	mov    %r13,%rbp
   a2951:	lea    0x29(%rsp),%rdx
   a2956:	mov    -0x1(%rdx),%rcx
   a295a:	lea    0xd9(%rsp),%rsi
   a2962:	mov    %rcx,0xf(%rsi)
   a2966:	movups -0x10(%rdx),%xmm0
   a296a:	movups %xmm0,(%rsi)
   a296d:	mov    %r12,0xd0(%rsp)
   a2975:	mov    %al,0xd8(%rsp)
   a297c:	movzbl 0xe8(%rsp),%r13d
   a2985:	lea    0x48(%rsp),%rdi
   a298a:	lea    0xd0(%rsp),%rsi
   a2992:	call   129370 <<alloc::vec::Vec<u64> as core::clone::Clone>::clone>
   a2997:	mov    0x58(%rsp),%rax
   a299c:	mov    %rax,0x20(%rsp)
   a29a1:	movups 0x48(%rsp),%xmm0
   a29a6:	movaps %xmm0,0x10(%rsp)
   a29ab:	mov    %r13b,0x28(%rsp)
   a29b0:	mov    %rbp,%rdi
   a29b3:	mov    %r14,%rsi
   a29b6:	mov    0xc(%rsp),%edx
   a29ba:	lea    0x10(%rsp),%rcx
   a29bf:	call   ada20 <<hyperreal::computable::node::Computable>::store_cache_value>
   a29c4:	cmp    $0xff,%al
   a29c6:	jne    a2e8f <<hyperreal::computable::node::Computable>::approx_signal+0x10ff>
   a29cc:	lea    0x210(%rsp),%rdi
   a29d4:	lea    0xd0(%rsp),%rsi
   a29dc:	call   a16b0 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::push>
   a29e1:	jmp    a22bf <<hyperreal::computable::node::Computable>::approx_signal+0x52f>
   a29e6:	cmp    $0x2,%eax
   a29e9:	je     a2e75 <<hyperreal::computable::node::Computable>::approx_signal+0x10e5>
   a29ef:	mov    %r9d,%ebp
   a29f2:	mov    %r15,%rdi
   a29f5:	lea    0x87304(%rip),%rsi        # 129d00 <std::sys::thread_local::native::eager::destroy::<core::cell::RefCell<core::option::Option<alloc::rc::Rc<hyperreal::computable::budget::ActiveBudget>>>>>
   a29fc:	call   *0xe3d36(%rip)        # 186738 <_DYNAMIC+0x3b0>
   a2a02:	movb   $0x0,0x10(%r15)
   a2a07:	mov    %ebp,%r9d
   a2a0a:	mov    (%r15),%rcx
   a2a0d:	movabs $0x7fffffffffffffff,%rax
   a2a17:	cmp    %rax,%rcx
   a2a1a:	jb     a237a <<hyperreal::computable::node::Computable>::approx_signal+0x5ea>
   a2a20:	lea    0xe0f91(%rip),%rdi        # 1839b8 <anon.953f3db8f5a996f677fb6a53175227a4.23.llvm.13083929455518204517>
   a2a27:	call   *0xe3d13(%rip)        # 186740 <_DYNAMIC+0x3b8>
   a2a2d:	jmp    a2f68 <<hyperreal::computable::node::Computable>::approx_signal+0x11d8>
   a2a32:	data16 data16 data16 data16 cs nopw 0x0(%rax,%rax,1)
   a2a40:	movb   $0x1,0x20(%rdx)
   a2a44:	mov    0x18(%rdx),%rdx
   a2a48:	test   %rdx,%rdx
   a2a4b:	jne    a2a40 <<hyperreal::computable::node::Computable>::approx_signal+0xcb0>
   a2a4d:	mov    %rcx,(%r15)
   a2a50:	movb   $0xc,0x8(%rbx)
   a2a54:	movq   $0xffffffffffffffff,(%rbx)
   a2a5b:	lea    0x210(%rsp),%rdi
   a2a63:	call   7dcf0 <core::ptr::drop_glue::<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>>
   a2a68:	mov    0xf0(%rsp),%rsi
   a2a70:	test   %rsi,%rsi
   a2a73:	je     a1f6b <<hyperreal::computable::node::Computable>::approx_signal+0x1db>
   a2a79:	mov    0xf8(%rsp),%rdi
   a2a81:	shl    $0x4,%rsi
   a2a85:	jmp    a2b41 <<hyperreal::computable::node::Computable>::approx_signal+0xdb1>
   a2a8a:	test   %al,%al
   a2a8c:	je     a2b59 <<hyperreal::computable::node::Computable>::approx_signal+0xdc9>
   a2a92:	xor    %esi,%esi
   a2a94:	lea    0x10(%rsp),%rdi
   a2a99:	call   c09b0 <<num_bigint::bigint::BigInt as core::convert::From<i64>>::from>
   a2a9e:	add    $0x38,%r12
   a2aa2:	mov    0x28(%rsp),%esi
   a2aa6:	mov    %r12,%rdi
   a2aa9:	call   bba90 <<hyperreal::computable::node::AtomicFacts>::publish_exact_sign>
   a2aae:	lea    0xf0(%rsp),%r15
   a2ab6:	lea    0x10(%rsp),%rsi
   a2abb:	mov    %r15,%rdi
   a2abe:	mov    %ebp,%edx
   a2ac0:	call   bc390 <hyperreal::computable::node::scale_at_precision>
   a2ac5:	movzbl 0x108(%rsp),%r12d
   a2ace:	lea    0x48(%rsp),%rdi
   a2ad3:	mov    %r15,%rsi
   a2ad6:	call   129370 <<alloc::vec::Vec<u64> as core::clone::Clone>::clone>
   a2adb:	mov    0x58(%rsp),%rax
   a2ae0:	mov    %rax,0x220(%rsp)
   a2ae8:	movups 0x48(%rsp),%xmm0
   a2aed:	movaps %xmm0,0x210(%rsp)
   a2af5:	mov    %r12b,0x228(%rsp)
   a2afd:	lea    0x210(%rsp),%rcx
   a2b05:	mov    %r13,%rdi
   a2b08:	mov    %r14,%rsi
   a2b0b:	mov    %ebp,%edx
   a2b0d:	call   ada20 <<hyperreal::computable::node::Computable>::store_cache_value>
   a2b12:	cmp    $0xff,%al
   a2b14:	je     a2d17 <<hyperreal::computable::node::Computable>::approx_signal+0xf87>
   a2b1a:	mov    %al,0x8(%rbx)
   a2b1d:	movq   $0xffffffffffffffff,(%rbx)
   a2b24:	mov    0xf0(%rsp),%rsi
   a2b2c:	test   %rsi,%rsi
   a2b2f:	je     a1f6b <<hyperreal::computable::node::Computable>::approx_signal+0x1db>
   a2b35:	mov    0xf8(%rsp),%rdi
   a2b3d:	shl    $0x3,%rsi
   a2b41:	mov    $0x8,%edx
   a2b46:	call   *0xe3a7c(%rip)        # 1865c8 <_DYNAMIC+0x240>
   a2b4c:	jmp    a1f6b <<hyperreal::computable::node::Computable>::approx_signal+0x1db>
   a2b51:	test   %cl,%cl
   a2b53:	jne    a2a92 <<hyperreal::computable::node::Computable>::approx_signal+0xd02>
   a2b59:	movb   $0x2,0x8(%rbx)
   a2b5d:	jmp    a1f64 <<hyperreal::computable::node::Computable>::approx_signal+0x1d4>
   a2b62:	movzbl 0x218(%rsp),%eax
   a2b6a:	jmp    a1e64 <<hyperreal::computable::node::Computable>::approx_signal+0xd4>
   a2b6f:	lea    0x210(%rsp),%rdi
   a2b77:	mov    0x40(%rsp),%rdx
   a2b7c:	mov    %ebp,%ecx
   a2b7e:	call   ea400 <<hyperreal::computable::approximation::Approximation>::approximate>
   a2b83:	mov    0x210(%rsp),%r15
   a2b8b:	movzbl 0x218(%rsp),%eax
   a2b93:	cmp    $0xffffffffffffffff,%r15
   a2b97:	je     a1e64 <<hyperreal::computable::node::Computable>::approx_signal+0xd4>
   a2b9d:	mov    0x228(%rsp),%rcx
   a2ba5:	mov    %rcx,0x108(%rsp)
   a2bad:	movups 0x219(%rsp),%xmm0
   a2bb5:	movups %xmm0,0xf9(%rsp)
   a2bbd:	mov    %r15,0xf0(%rsp)
   a2bc5:	mov    %al,0xf8(%rsp)
   a2bcc:	movzbl 0x108(%rsp),%r12d
   a2bd5:	lea    0x10(%rsp),%rdi
   a2bda:	lea    0xf0(%rsp),%rsi
   a2be2:	call   129370 <<alloc::vec::Vec<u64> as core::clone::Clone>::clone>
   a2be7:	mov    0x20(%rsp),%rax
   a2bec:	mov    %rax,0x220(%rsp)
   a2bf4:	movups 0x10(%rsp),%xmm0
   a2bf9:	movaps %xmm0,0x210(%rsp)
   a2c01:	mov    %r12b,0x228(%rsp)
   a2c09:	lea    0x210(%rsp),%rcx
   a2c11:	mov    %r13,%rdi
   a2c14:	mov    %r14,%rsi
   a2c17:	mov    %ebp,%edx
   a2c19:	call   ada20 <<hyperreal::computable::node::Computable>::store_cache_value>
   a2c1e:	cmp    $0xff,%al
   a2c20:	je     a2d17 <<hyperreal::computable::node::Computable>::approx_signal+0xf87>
   a2c26:	mov    %al,0x8(%rbx)
   a2c29:	movq   $0xffffffffffffffff,(%rbx)
   a2c30:	test   %r15,%r15
   a2c33:	je     a1f6b <<hyperreal::computable::node::Computable>::approx_signal+0x1db>
   a2c39:	mov    0xf8(%rsp),%rdi
   a2c41:	shl    $0x3,%r15
   a2c45:	mov    $0x8,%edx
   a2c4a:	mov    %r15,%rsi
   a2c4d:	call   *0xe3975(%rip)        # 1865c8 <_DYNAMIC+0x240>
   a2c53:	jmp    a1f6b <<hyperreal::computable::node::Computable>::approx_signal+0x1db>
   a2c58:	movzbl 0x218(%rsp),%eax
   a2c60:	mov    0x228(%rsp),%rcx
   a2c68:	mov    %rcx,0x108(%rsp)
   a2c70:	movups 0x219(%rsp),%xmm0
   a2c78:	movups %xmm0,0xf9(%rsp)
   a2c80:	mov    %r12,0xf0(%rsp)
   a2c88:	mov    %al,0xf8(%rsp)
   a2c8f:	movzbl 0x108(%rsp),%r15d
   a2c98:	lea    0x10(%rsp),%rdi
   a2c9d:	lea    0xf0(%rsp),%rsi
   a2ca5:	call   129370 <<alloc::vec::Vec<u64> as core::clone::Clone>::clone>
   a2caa:	mov    0x20(%rsp),%rax
   a2caf:	mov    %rax,0x220(%rsp)
   a2cb7:	movups 0x10(%rsp),%xmm0
   a2cbc:	movaps %xmm0,0x210(%rsp)
   a2cc4:	mov    %r15b,0x228(%rsp)
   a2ccc:	lea    0x210(%rsp),%rcx
   a2cd4:	mov    %r13,%rdi
   a2cd7:	mov    %r14,%rsi
   a2cda:	mov    %ebp,%edx
   a2cdc:	call   ada20 <<hyperreal::computable::node::Computable>::store_cache_value>
   a2ce1:	cmp    $0xff,%al
   a2ce3:	je     a2d17 <<hyperreal::computable::node::Computable>::approx_signal+0xf87>
   a2ce5:	mov    %al,0x8(%rbx)
   a2ce8:	movq   $0xffffffffffffffff,(%rbx)
   a2cef:	test   %r12,%r12
   a2cf2:	je     a1f6b <<hyperreal::computable::node::Computable>::approx_signal+0x1db>
   a2cf8:	mov    0xf8(%rsp),%rdi
   a2d00:	shl    $0x3,%r12
   a2d04:	mov    $0x8,%edx
   a2d09:	mov    %r12,%rsi
   a2d0c:	call   *0xe38b6(%rip)        # 1865c8 <_DYNAMIC+0x240>
   a2d12:	jmp    a1f6b <<hyperreal::computable::node::Computable>::approx_signal+0x1db>
   a2d17:	movups 0xf0(%rsp),%xmm0
   a2d1f:	movups 0x100(%rsp),%xmm1
   a2d27:	jmp    a1ffa <<hyperreal::computable::node::Computable>::approx_signal+0x26a>
   a2d2c:	lea    0x10(%rsp),%rdi
   a2d31:	lea    0x210(%rsp),%rsi
   a2d39:	call   a1620 <<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>::pop>
   a2d3e:	cmpq   $0xffffffffffffffff,0x10(%rsp)
   a2d44:	je     a2f0c <<hyperreal::computable::node::Computable>::approx_signal+0x117c>
   a2d4a:	movups 0x10(%rsp),%xmm0
   a2d4f:	movups 0x20(%rsp),%xmm1
   a2d54:	movups %xmm1,0x10(%rbx)
   a2d58:	movups %xmm0,(%rbx)
   a2d5b:	jmp    a2a5b <<hyperreal::computable::node::Computable>::approx_signal+0xccb>
   a2d60:	cmp    $0x1,%eax
   a2d63:	jne    a2d9d <<hyperreal::computable::node::Computable>::approx_signal+0x100d>
   a2d65:	lea    0x86f94(%rip),%rsi        # 129d00 <std::sys::thread_local::native::eager::destroy::<core::cell::RefCell<core::option::Option<alloc::rc::Rc<hyperreal::computable::budget::ActiveBudget>>>>>
   a2d6c:	mov    %r15,%rdi
   a2d6f:	call   *0xe39c3(%rip)        # 186738 <_DYNAMIC+0x3b0>
   a2d75:	movb   $0x0,0x10(%r15)
   a2d7a:	movabs $0x7fffffffffffffff,%rax
   a2d84:	mov    (%r15),%rcx
   a2d87:	cmp    %rax,%rcx
   a2d8a:	jb     a1dfb <<hyperreal::computable::node::Computable>::approx_signal+0x6b>
   a2d90:	lea    0xe0c39(%rip),%rdi        # 1839d0 <anon.953f3db8f5a996f677fb6a53175227a4.24.llvm.13083929455518204517>
   a2d97:	call   *0xe39a3(%rip)        # 186740 <_DYNAMIC+0x3b8>
   a2d9d:	lea    0xe0b64(%rip),%rdi        # 183908 <anon.953f3db8f5a996f677fb6a53175227a4.4.llvm.13083929455518204517>
   a2da4:	call   *0xe399e(%rip)        # 186748 <_DYNAMIC+0x3c0>
   a2daa:	mov    %al,0x8(%rbx)
   a2dad:	movq   $0xffffffffffffffff,(%rbx)
   a2db4:	mov    0x90(%rsp),%rsi
   a2dbc:	test   %rsi,%rsi
   a2dbf:	je     a2a5b <<hyperreal::computable::node::Computable>::approx_signal+0xccb>
   a2dc5:	mov    0x98(%rsp),%rdi
   a2dcd:	jmp    a2e11 <<hyperreal::computable::node::Computable>::approx_signal+0x1081>
   a2dcf:	mov    %al,0x8(%rbx)
   a2dd2:	movq   $0xffffffffffffffff,(%rbx)
   a2dd9:	mov    0x70(%rsp),%rsi
   a2dde:	test   %rsi,%rsi
   a2de1:	je     a2a5b <<hyperreal::computable::node::Computable>::approx_signal+0xccb>
   a2de7:	mov    0x78(%rsp),%rdi
   a2dec:	jmp    a2e11 <<hyperreal::computable::node::Computable>::approx_signal+0x1081>
   a2dee:	mov    %al,0x8(%rbx)
   a2df1:	movq   $0xffffffffffffffff,(%rbx)
   a2df8:	mov    0x330(%rsp),%rsi
   a2e00:	test   %rsi,%rsi
   a2e03:	je     a2a5b <<hyperreal::computable::node::Computable>::approx_signal+0xccb>
   a2e09:	mov    0x338(%rsp),%rdi
   a2e11:	shl    $0x3,%rsi
   a2e15:	mov    $0x8,%edx
   a2e1a:	call   *0xe37a8(%rip)        # 1865c8 <_DYNAMIC+0x240>
   a2e20:	jmp    a2a5b <<hyperreal::computable::node::Computable>::approx_signal+0xccb>
   a2e25:	neg    %edx
   a2e27:	setno  %al
   a2e2a:	add    %al,%al
   a2e2c:	add    $0xa,%al
   a2e2e:	mov    %al,0x8(%rbx)
   a2e31:	jmp    a2a54 <<hyperreal::computable::node::Computable>::approx_signal+0xcc4>
   a2e36:	movzbl 0x18(%rsp),%eax
   a2e3b:	mov    %al,0x8(%rbx)
   a2e3e:	jmp    a2a54 <<hyperreal::computable::node::Computable>::approx_signal+0xcc4>
   a2e43:	mov    %al,0x8(%rbx)
   a2e46:	movq   $0xffffffffffffffff,(%rbx)
   a2e4d:	test   %r12,%r12
   a2e50:	je     a2a5b <<hyperreal::computable::node::Computable>::approx_signal+0xccb>
   a2e56:	mov    0xb8(%rsp),%rdi
   a2e5e:	shl    $0x3,%r12
   a2e62:	mov    $0x8,%edx
   a2e67:	mov    %r12,%rsi
   a2e6a:	call   *0xe3758(%rip)        # 1865c8 <_DYNAMIC+0x240>
   a2e70:	jmp    a2a5b <<hyperreal::computable::node::Computable>::approx_signal+0xccb>
   a2e75:	lea    0xe0a8c(%rip),%rdi        # 183908 <anon.953f3db8f5a996f677fb6a53175227a4.4.llvm.13083929455518204517>
   a2e7c:	call   *0xe38c6(%rip)        # 186748 <_DYNAMIC+0x3c0>
   a2e82:	jmp    a2f68 <<hyperreal::computable::node::Computable>::approx_signal+0x11d8>
   a2e87:	mov    %al,0x8(%rbx)
   a2e8a:	jmp    a2a54 <<hyperreal::computable::node::Computable>::approx_signal+0xcc4>
   a2e8f:	mov    %al,0x8(%rbx)
   a2e92:	movq   $0xffffffffffffffff,(%rbx)
   a2e99:	test   %r12,%r12
   a2e9c:	je     a2a5b <<hyperreal::computable::node::Computable>::approx_signal+0xccb>
   a2ea2:	mov    0xd8(%rsp),%rdi
   a2eaa:	jmp    a2e5e <<hyperreal::computable::node::Computable>::approx_signal+0x10ce>
   a2eac:	mov    $0x20,%esi
   a2eb1:	lea    -0x6e568(%rip),%rax        # 34950 <anon.7b75c566c6a805bace2b01974d435009.54.llvm.17861709745643738061+0xac>
   a2eb8:	mov    %rax,0x30(%rsp)
   a2ebd:	lea    0xdf7c4(%rip),%rax        # 182688 <anon.7b75c566c6a805bace2b01974d435009.258.llvm.17861709745643738061+0x78>
   a2ec4:	mov    %rax,0x38(%rsp)
   a2ec9:	jmp    a2f29 <<hyperreal::computable::node::Computable>::approx_signal+0x1199>
   a2ecb:	mov    $0x1b,%esi
   a2ed0:	lea    -0x6d9d1(%rip),%rax        # 35506 <anon.7b75c566c6a805bace2b01974d435009.257.llvm.17861709745643738061+0x6b>
   a2ed7:	mov    %rax,0x30(%rsp)
   a2edc:	lea    0xdf775(%rip),%rax        # 182658 <anon.7b75c566c6a805bace2b01974d435009.258.llvm.17861709745643738061+0x48>
   a2ee3:	mov    %rax,0x38(%rsp)
   a2ee8:	jmp    a2f29 <<hyperreal::computable::node::Computable>::approx_signal+0x1199>
   a2eea:	mov    $0x20,%esi
   a2eef:	jmp    a2f29 <<hyperreal::computable::node::Computable>::approx_signal+0x1199>
   a2ef1:	lea    -0x6d9d7(%rip),%rdi        # 35521 <anon.7b75c566c6a805bace2b01974d435009.257.llvm.17861709745643738061+0x86>
   a2ef8:	lea    0xdf771(%rip),%rdx        # 182670 <anon.7b75c566c6a805bace2b01974d435009.258.llvm.17861709745643738061+0x60>
   a2eff:	mov    $0x1b,%esi
   a2f04:	call   *0xe391e(%rip)        # 186828 <_DYNAMIC+0x4a0>
   a2f0a:	jmp    a2f68 <<hyperreal::computable::node::Computable>::approx_signal+0x11d8>
   a2f0c:	lea    0xdf78d(%rip),%rax        # 1826a0 <anon.7b75c566c6a805bace2b01974d435009.258.llvm.17861709745643738061+0x90>
   a2f13:	mov    %rax,0x38(%rsp)
   a2f18:	mov    $0x22,%esi
   a2f1d:	lea    -0x6d9e8(%rip),%rax        # 3553c <anon.7b75c566c6a805bace2b01974d435009.257.llvm.17861709745643738061+0xa1>
   a2f24:	mov    %rax,0x30(%rsp)
   a2f29:	mov    0x30(%rsp),%rdi
   a2f2e:	mov    0x38(%rsp),%rdx
   a2f33:	call   *0xe38ef(%rip)        # 186828 <_DYNAMIC+0x4a0>
   a2f39:	jmp    a2f68 <<hyperreal::computable::node::Computable>::approx_signal+0x11d8>
   a2f3b:	movb   $0xa,0x8(%rbx)
   a2f3f:	jmp    a2a54 <<hyperreal::computable::node::Computable>::approx_signal+0xcc4>
   a2f44:	lea    0xdf9c5(%rip),%rdx        # 182910 <anon.7b75c566c6a805bace2b01974d435009.258.llvm.17861709745643738061+0x300>
   a2f4b:	mov    $0x4,%esi
   a2f50:	call   *0xe36b2(%rip)        # 186608 <_DYNAMIC+0x280>
   a2f56:	lea    0xdf6cb(%rip),%rdx        # 182628 <anon.7b75c566c6a805bace2b01974d435009.258.llvm.17861709745643738061+0x18>
   a2f5d:	mov    $0x10,%esi
   a2f62:	call   *0xe36a0(%rip)        # 186608 <_DYNAMIC+0x280>
   a2f68:	ud2
   a2f6a:	jmp    a30ca <<hyperreal::computable::node::Computable>::approx_signal+0x133a>
   a2f6f:	mov    %rax,%rbx
   a2f72:	test   %r12,%r12
   a2f75:	je     a30cd <<hyperreal::computable::node::Computable>::approx_signal+0x133d>
   a2f7b:	mov    0xd8(%rsp),%rdi
   a2f83:	jmp    a2f9e <<hyperreal::computable::node::Computable>::approx_signal+0x120e>
   a2f85:	jmp    a30ca <<hyperreal::computable::node::Computable>::approx_signal+0x133a>
   a2f8a:	mov    %rax,%rbx
   a2f8d:	test   %r12,%r12
   a2f90:	je     a30cd <<hyperreal::computable::node::Computable>::approx_signal+0x133d>
   a2f96:	mov    0xb8(%rsp),%rdi
   a2f9e:	shl    $0x3,%r12
   a2fa2:	mov    $0x8,%edx
   a2fa7:	jmp    a30bd <<hyperreal::computable::node::Computable>::approx_signal+0x132d>
   a2fac:	jmp    a30ca <<hyperreal::computable::node::Computable>::approx_signal+0x133a>
   a2fb1:	jmp    a30ca <<hyperreal::computable::node::Computable>::approx_signal+0x133a>
   a2fb6:	jmp    a30ca <<hyperreal::computable::node::Computable>::approx_signal+0x133a>
   a2fbb:	jmp    a30a9 <<hyperreal::computable::node::Computable>::approx_signal+0x1319>
   a2fc0:	mov    %rax,%rbx
   a2fc3:	mov    0x330(%rsp),%rsi
   a2fcb:	test   %rsi,%rsi
   a2fce:	je     a30cd <<hyperreal::computable::node::Computable>::approx_signal+0x133d>
   a2fd4:	mov    0x338(%rsp),%rdi
   a2fdc:	jmp    a3017 <<hyperreal::computable::node::Computable>::approx_signal+0x1287>
   a2fde:	jmp    a30ca <<hyperreal::computable::node::Computable>::approx_signal+0x133a>
   a2fe3:	mov    %rax,%rbx
   a2fe6:	mov    0x70(%rsp),%rsi
   a2feb:	test   %rsi,%rsi
   a2fee:	je     a30cd <<hyperreal::computable::node::Computable>::approx_signal+0x133d>
   a2ff4:	mov    0x78(%rsp),%rdi
   a2ff9:	jmp    a3017 <<hyperreal::computable::node::Computable>::approx_signal+0x1287>
   a2ffb:	mov    %rax,%rbx
   a2ffe:	mov    0x90(%rsp),%rsi
   a3006:	test   %rsi,%rsi
   a3009:	je     a30cd <<hyperreal::computable::node::Computable>::approx_signal+0x133d>
   a300f:	mov    0x98(%rsp),%rdi
   a3017:	shl    $0x3,%rsi
   a301b:	mov    $0x8,%edx
   a3020:	call   *0xe35a2(%rip)        # 1865c8 <_DYNAMIC+0x240>
   a3026:	jmp    a30cd <<hyperreal::computable::node::Computable>::approx_signal+0x133d>
   a302b:	mov    %rax,%rbx
   a302e:	test   %r15,%r15
   a3031:	je     a30fe <<hyperreal::computable::node::Computable>::approx_signal+0x136e>
   a3037:	mov    0xf8(%rsp),%rdi
   a303f:	shl    $0x3,%r15
   a3043:	mov    $0x8,%edx
   a3048:	mov    %r15,%rsi
   a304b:	jmp    a30f8 <<hyperreal::computable::node::Computable>::approx_signal+0x1368>
   a3050:	mov    %rax,%rbx
   a3053:	test   %r12,%r12
   a3056:	je     a30fe <<hyperreal::computable::node::Computable>::approx_signal+0x136e>
   a305c:	mov    0xf8(%rsp),%rdi
   a3064:	shl    $0x3,%r12
   a3068:	mov    $0x8,%edx
   a306d:	mov    %r12,%rsi
   a3070:	jmp    a30f8 <<hyperreal::computable::node::Computable>::approx_signal+0x1368>
   a3075:	jmp    a30ca <<hyperreal::computable::node::Computable>::approx_signal+0x133a>
   a3077:	mov    %rax,%rbx
   a307a:	mov    0x10(%rsp),%rsi
   a307f:	test   %rsi,%rsi
   a3082:	je     a30fe <<hyperreal::computable::node::Computable>::approx_signal+0x136e>
   a3084:	mov    0x18(%rsp),%rdi
   a3089:	jmp    a30a3 <<hyperreal::computable::node::Computable>::approx_signal+0x1313>
   a308b:	mov    %rax,%rbx
   a308e:	mov    0xf0(%rsp),%rsi
   a3096:	test   %rsi,%rsi
   a3099:	je     a30fe <<hyperreal::computable::node::Computable>::approx_signal+0x136e>
   a309b:	mov    0xf8(%rsp),%rdi
   a30a3:	shl    $0x3,%rsi
   a30a7:	jmp    a30f3 <<hyperreal::computable::node::Computable>::approx_signal+0x1363>
   a30a9:	mov    %rax,%rbx
   a30ac:	test   %r12,%r12
   a30af:	je     a30cd <<hyperreal::computable::node::Computable>::approx_signal+0x133d>
   a30b1:	shl    $0x3,%r12
   a30b5:	mov    $0x8,%edx
   a30ba:	mov    %r13,%rdi
   a30bd:	mov    %r12,%rsi
   a30c0:	call   *0xe3502(%rip)        # 1865c8 <_DYNAMIC+0x240>
   a30c6:	jmp    a30cd <<hyperreal::computable::node::Computable>::approx_signal+0x133d>
   a30c8:	jmp    a30ca <<hyperreal::computable::node::Computable>::approx_signal+0x133a>
   a30ca:	mov    %rax,%rbx
   a30cd:	lea    0x210(%rsp),%rdi
   a30d5:	call   7dcf0 <core::ptr::drop_glue::<hyperreal::computable::node::InlineStack<num_bigint::bigint::BigInt, 8>>>
   a30da:	mov    0xf0(%rsp),%rsi
   a30e2:	test   %rsi,%rsi
   a30e5:	je     a30fe <<hyperreal::computable::node::Computable>::approx_signal+0x136e>
   a30e7:	mov    0xf8(%rsp),%rdi
   a30ef:	shl    $0x4,%rsi
   a30f3:	mov    $0x8,%edx
   a30f8:	call   *0xe34ca(%rip)        # 1865c8 <_DYNAMIC+0x240>
   a30fe:	mov    %rbx,%rdi
   a3101:	call   180b10 <_Unwind_Resume@plt>
