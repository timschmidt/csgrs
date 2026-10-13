
/home/tim/Documents/GitHub/workspace/csgrs/target/outlined-same-radicand-products/cost-v2-binaries/baseline-quadratic-time:     file format elf64-x86-64


Disassembly of section .text:

00000000000cef30 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply>:
   cef30:	55                   	push   %rbp
   cef31:	41 57                	push   %r15
   cef33:	41 56                	push   %r14
   cef35:	41 55                	push   %r13
   cef37:	41 54                	push   %r12
   cef39:	53                   	push   %rbx
   cef3a:	48 81 ec d8 01 00 00 	sub    $0x1d8,%rsp
   cef41:	49 89 fe             	mov    %rdi,%r14
   cef44:	48 c7 44 24 40 00 00 	movq   $0x0,0x40(%rsp)
   cef4b:	00 00 
   cef4d:	48 c7 44 24 48 08 00 	movq   $0x8,0x48(%rsp)
   cef54:	00 00 
   cef56:	48 c7 44 24 50 00 00 	movq   $0x0,0x50(%rsp)
   cef5d:	00 00 
   cef5f:	48 8b 42 08          	mov    0x8(%rdx),%rax
   cef63:	48 89 04 24          	mov    %rax,(%rsp)
   cef67:	48 89 54 24 58       	mov    %rdx,0x58(%rsp)
   cef6c:	4c 8b 62 10          	mov    0x10(%rdx),%r12
   cef70:	49 83 fc 01          	cmp    $0x1,%r12
   cef74:	0f 85 b8 01 00 00    	jne    cf132 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x202>
   cef7a:	48 8b 06             	mov    (%rsi),%rax
   cef7d:	48 8b 5e 08          	mov    0x8(%rsi),%rbx
   cef81:	48 8b 4e 10          	mov    0x10(%rsi),%rcx
   cef85:	49 89 cd             	mov    %rcx,%r13
   cef88:	49 c1 e5 06          	shl    $0x6,%r13
   cef8c:	4a 8d 14 2b          	lea    (%rbx,%r13,1),%rdx
   cef90:	48 89 5c 24 18       	mov    %rbx,0x18(%rsp)
   cef95:	48 89 5c 24 20       	mov    %rbx,0x20(%rsp)
   cef9a:	48 89 44 24 28       	mov    %rax,0x28(%rsp)
   cef9f:	48 89 54 24 30       	mov    %rdx,0x30(%rsp)
   cefa4:	48 85 c9             	test   %rcx,%rcx
   cefa7:	0f 84 64 01 00 00    	je     cf111 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x1e1>
   cefad:	48 89 54 24 60       	mov    %rdx,0x60(%rsp)
   cefb2:	4c 89 74 24 10       	mov    %r14,0x10(%rsp)
   cefb7:	48 8b 04 24          	mov    (%rsp),%rax
   cefbb:	48 83 c0 08          	add    $0x8,%rax
   cefbf:	48 89 44 24 38       	mov    %rax,0x38(%rsp)
   cefc4:	4c 8d bc 24 98 00 00 	lea    0x98(%rsp),%r15
   cefcb:	00 
   cefcc:	48 83 c3 40          	add    $0x40,%rbx
   cefd0:	48 8d ac 24 00 01 00 	lea    0x100(%rsp),%rbp
   cefd7:	00 
   cefd8:	0f 1f 84 00 00 00 00 	nopl   0x0(%rax,%rax,1)
   cefdf:	00 
   cefe0:	48 8b 43 c8          	mov    -0x38(%rbx),%rax
   cefe4:	48 83 f8 ff          	cmp    $0xffffffffffffffff,%rax
   cefe8:	0f 84 19 01 00 00    	je     cf107 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x1d7>
   cefee:	44 0f b7 73 c0       	movzwl -0x40(%rbx),%r14d
   ceff3:	48 89 84 24 90 00 00 	mov    %rax,0x90(%rsp)
   ceffa:	00 
   ceffb:	0f 10 43 d0          	movups -0x30(%rbx),%xmm0
   cefff:	0f 10 4b e0          	movups -0x20(%rbx),%xmm1
   cf003:	0f 10 53 f0          	movups -0x10(%rbx),%xmm2
   cf007:	41 0f 11 57 20       	movups %xmm2,0x20(%r15)
   cf00c:	41 0f 11 4f 10       	movups %xmm1,0x10(%r15)
   cf011:	41 0f 11 07          	movups %xmm0,(%r15)
   cf015:	48 8b 04 24          	mov    (%rsp),%rax
   cf019:	66 44 03 30          	add    (%rax),%r14w
   cf01d:	0f 80 7b 06 00 00    	jo     cf69e <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x76e>
   cf023:	48 8b 84 24 c0 00 00 	mov    0xc0(%rsp),%rax
   cf02a:	00 
   cf02b:	48 89 84 24 30 01 00 	mov    %rax,0x130(%rsp)
   cf032:	00 
   cf033:	48 8b 84 24 90 00 00 	mov    0x90(%rsp),%rax
   cf03a:	00 
   cf03b:	48 8b 8c 24 98 00 00 	mov    0x98(%rsp),%rcx
   cf042:	00 
   cf043:	0f 10 84 24 a0 00 00 	movups 0xa0(%rsp),%xmm0
   cf04a:	00 
   cf04b:	0f 10 8c 24 b0 00 00 	movups 0xb0(%rsp),%xmm1
   cf052:	00 
   cf053:	0f 29 8c 24 20 01 00 	movaps %xmm1,0x120(%rsp)
   cf05a:	00 
   cf05b:	0f 29 84 24 10 01 00 	movaps %xmm0,0x110(%rsp)
   cf062:	00 
   cf063:	48 89 84 24 00 01 00 	mov    %rax,0x100(%rsp)
   cf06a:	00 
   cf06b:	48 89 8c 24 08 01 00 	mov    %rcx,0x108(%rsp)
   cf072:	00 
   cf073:	48 8d bc 24 c8 00 00 	lea    0xc8(%rsp),%rdi
   cf07a:	00 
   cf07b:	48 89 ee             	mov    %rbp,%rsi
   cf07e:	48 8b 54 24 38       	mov    0x38(%rsp),%rdx
   cf083:	e8 c8 df ff ff       	call   cd050 <<hyperreal::computable::node::RadicalCoefficient>::multiply>
   cf088:	48 83 bc 24 c8 00 00 	cmpq   $0xffffffffffffffff,0xc8(%rsp)
   cf08f:	00 ff 
   cf091:	0f 84 08 05 00 00    	je     cf59f <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x66f>
   cf097:	48 8b 84 24 f8 00 00 	mov    0xf8(%rsp),%rax
   cf09e:	00 
   cf09f:	48 89 84 24 d0 01 00 	mov    %rax,0x1d0(%rsp)
   cf0a6:	00 
   cf0a7:	0f 10 84 24 c8 00 00 	movups 0xc8(%rsp),%xmm0
   cf0ae:	00 
   cf0af:	0f 10 8c 24 d8 00 00 	movups 0xd8(%rsp),%xmm1
   cf0b6:	00 
   cf0b7:	0f 10 94 24 e8 00 00 	movups 0xe8(%rsp),%xmm2
   cf0be:	00 
   cf0bf:	0f 29 94 24 c0 01 00 	movaps %xmm2,0x1c0(%rsp)
   cf0c6:	00 
   cf0c7:	0f 29 8c 24 b0 01 00 	movaps %xmm1,0x1b0(%rsp)
   cf0ce:	00 
   cf0cf:	0f 29 84 24 a0 01 00 	movaps %xmm0,0x1a0(%rsp)
   cf0d6:	00 
   cf0d7:	48 8d 7c 24 40       	lea    0x40(%rsp),%rdi
   cf0dc:	44 89 f6             	mov    %r14d,%esi
   cf0df:	48 8d 94 24 a0 01 00 	lea    0x1a0(%rsp),%rdx
   cf0e6:	00 
   cf0e7:	e8 b4 f2 ff ff       	call   ce3a0 <<hyperreal::computable::node::PiLaurentPolynomial>::insert>
   cf0ec:	84 c0                	test   %al,%al
   cf0ee:	0f 84 ab 04 00 00    	je     cf59f <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x66f>
   cf0f4:	48 83 c3 40          	add    $0x40,%rbx
   cf0f8:	49 83 c5 c0          	add    $0xffffffffffffffc0,%r13
   cf0fc:	0f 85 de fe ff ff    	jne    cefe0 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0xb0>
   cf102:	48 8b 5c 24 60       	mov    0x60(%rsp),%rbx
   cf107:	48 89 5c 24 20       	mov    %rbx,0x20(%rsp)
   cf10c:	4c 8b 74 24 10       	mov    0x10(%rsp),%r14
   cf111:	48 8d 7c 24 18       	lea    0x18(%rsp),%rdi
   cf116:	e8 45 e1 03 00       	call   10d260 <<alloc::vec::into_iter::IntoIter<(i16, hyperreal::computable::node::RadicalCoefficient)> as core::ops::drop::Drop>::drop>
   cf11b:	48 8b 44 24 50       	mov    0x50(%rsp),%rax
   cf120:	49 89 46 10          	mov    %rax,0x10(%r14)
   cf124:	0f 10 44 24 40       	movups 0x40(%rsp),%xmm0
   cf129:	41 0f 11 06          	movups %xmm0,(%r14)
   cf12d:	e9 03 05 00 00       	jmp    cf635 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x705>
   cf132:	48 8b 0e             	mov    (%rsi),%rcx
   cf135:	48 8b 46 08          	mov    0x8(%rsi),%rax
   cf139:	48 8b 56 10          	mov    0x10(%rsi),%rdx
   cf13d:	48 89 d6             	mov    %rdx,%rsi
   cf140:	48 c1 e6 06          	shl    $0x6,%rsi
   cf144:	48 01 c6             	add    %rax,%rsi
   cf147:	48 89 44 24 18       	mov    %rax,0x18(%rsp)
   cf14c:	48 89 44 24 20       	mov    %rax,0x20(%rsp)
   cf151:	48 89 4c 24 28       	mov    %rcx,0x28(%rsp)
   cf156:	48 89 b4 24 88 00 00 	mov    %rsi,0x88(%rsp)
   cf15d:	00 
   cf15e:	48 89 74 24 30       	mov    %rsi,0x30(%rsp)
   cf163:	48 85 d2             	test   %rdx,%rdx
   cf166:	0f 84 d1 03 00 00    	je     cf53d <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x60d>
   cf16c:	48 8d b4 24 98 00 00 	lea    0x98(%rsp),%rsi
   cf173:	00 
   cf174:	4c 89 e1             	mov    %r12,%rcx
   cf177:	48 c1 e1 06          	shl    $0x6,%rcx
   cf17b:	48 03 0c 24          	add    (%rsp),%rcx
   cf17f:	48 89 4c 24 60       	mov    %rcx,0x60(%rsp)
   cf184:	4c 89 74 24 10       	mov    %r14,0x10(%rsp)
   cf189:	48 8d 50 40          	lea    0x40(%rax),%rdx
   cf18d:	48 8b 48 08          	mov    0x8(%rax),%rcx
   cf191:	48 83 f9 ff          	cmp    $0xffffffffffffffff,%rcx
   cf195:	0f 84 9d 03 00 00    	je     cf538 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x608>
   cf19b:	48 89 54 24 08       	mov    %rdx,0x8(%rsp)
   cf1a0:	0f b7 10             	movzwl (%rax),%edx
   cf1a3:	66 89 54 24 38       	mov    %dx,0x38(%rsp)
   cf1a8:	48 89 8c 24 90 00 00 	mov    %rcx,0x90(%rsp)
   cf1af:	00 
   cf1b0:	0f 10 40 10          	movups 0x10(%rax),%xmm0
   cf1b4:	0f 10 48 20          	movups 0x20(%rax),%xmm1
   cf1b8:	0f 10 50 30          	movups 0x30(%rax),%xmm2
   cf1bc:	0f 11 56 20          	movups %xmm2,0x20(%rsi)
   cf1c0:	0f 11 4e 10          	movups %xmm1,0x10(%rsi)
   cf1c4:	0f 11 06             	movups %xmm0,(%rsi)
   cf1c7:	4d 85 e4             	test   %r12,%r12
   cf1ca:	0f 84 30 03 00 00    	je     cf500 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x5d0>
   cf1d0:	4c 8b b4 24 c0 00 00 	mov    0xc0(%rsp),%r14
   cf1d7:	00 
   cf1d8:	48 8b 9c 24 a0 00 00 	mov    0xa0(%rsp),%rbx
   cf1df:	00 
   cf1e0:	48 89 dd             	mov    %rbx,%rbp
   cf1e3:	48 c1 e5 04          	shl    $0x4,%rbp
   cf1e7:	48 89 d8             	mov    %rbx,%rax
   cf1ea:	48 c1 e8 3c          	shr    $0x3c,%rax
   cf1ee:	0f 94 c0             	sete   %al
   cf1f1:	48 b9 f9 ff ff ff ff 	movabs $0x7ffffffffffffff9,%rcx
   cf1f8:	ff ff 7f 
   cf1fb:	48 39 cd             	cmp    %rcx,%rbp
   cf1fe:	0f 92 c1             	setb   %cl
   cf201:	84 c8                	test   %cl,%al
   cf203:	0f 84 72 03 00 00    	je     cf57b <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x64b>
   cf209:	48 85 ed             	test   %rbp,%rbp
   cf20c:	0f 84 ab 01 00 00    	je     cf3bd <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x48d>
   cf212:	48 8b 84 24 98 00 00 	mov    0x98(%rsp),%rax
   cf219:	00 
   cf21a:	48 89 84 24 40 01 00 	mov    %rax,0x140(%rsp)
   cf221:	00 
   cf222:	4c 8b 3c 24          	mov    (%rsp),%r15
   cf226:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
   cf22d:	00 00 00 
   cf230:	45 0f b7 2f          	movzwl (%r15),%r13d
   cf234:	66 44 03 6c 24 38    	add    0x38(%rsp),%r13w
   cf23a:	0f 80 7c 03 00 00    	jo     cf5bc <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x68c>
   cf240:	f0 49 ff 06          	lock incq (%r14)
   cf244:	0f 8e 53 03 00 00    	jle    cf59d <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x66d>
   cf24a:	4c 89 74 24 68       	mov    %r14,0x68(%rsp)
   cf24f:	ff 15 73 f9 09 00    	call   *0x9f973(%rip)        # 16ebc8 <_DYNAMIC+0x338>
   cf255:	be 08 00 00 00       	mov    $0x8,%esi
   cf25a:	48 89 ef             	mov    %rbp,%rdi
   cf25d:	ff 15 6d f9 09 00    	call   *0x9f96d(%rip)        # 16ebd0 <_DYNAMIC+0x340>
   cf263:	48 85 c0             	test   %rax,%rax
   cf266:	0f 84 55 04 00 00    	je     cf6c1 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x791>
   cf26c:	48 85 db             	test   %rbx,%rbx
   cf26f:	74 48                	je     cf2b9 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x389>
   cf271:	31 c9                	xor    %ecx,%ecx
   cf273:	48 89 da             	mov    %rbx,%rdx
   cf276:	48 8b b4 24 40 01 00 	mov    0x140(%rsp),%rsi
   cf27d:	00 
   cf27e:	66 90                	xchg   %ax,%ax
   cf280:	48 39 cd             	cmp    %rcx,%rbp
   cf283:	74 34                	je     cf2b9 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x389>
   cf285:	48 8b 3e             	mov    (%rsi),%rdi
   cf288:	4c 8b 46 08          	mov    0x8(%rsi),%r8
   cf28c:	f0 48 ff 07          	lock incq (%rdi)
   cf290:	0f 8e 07 03 00 00    	jle    cf59d <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x66d>
   cf296:	f0 49 ff 00          	lock incq (%r8)
   cf29a:	0f 8e fd 02 00 00    	jle    cf59d <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x66d>
   cf2a0:	48 ff ca             	dec    %rdx
   cf2a3:	48 83 c6 10          	add    $0x10,%rsi
   cf2a7:	48 89 3c 08          	mov    %rdi,(%rax,%rcx,1)
   cf2ab:	4c 89 44 08 08       	mov    %r8,0x8(%rax,%rcx,1)
   cf2b0:	48 83 c1 10          	add    $0x10,%rcx
   cf2b4:	48 85 d2             	test   %rdx,%rdx
   cf2b7:	75 c7                	jne    cf280 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x350>
   cf2b9:	48 89 5c 24 70       	mov    %rbx,0x70(%rsp)
   cf2be:	48 89 44 24 78       	mov    %rax,0x78(%rsp)
   cf2c3:	48 89 9c 24 80 00 00 	mov    %rbx,0x80(%rsp)
   cf2ca:	00 
   cf2cb:	48 8d bc 24 48 01 00 	lea    0x148(%rsp),%rdi
   cf2d2:	00 
   cf2d3:	48 8d b4 24 a8 00 00 	lea    0xa8(%rsp),%rsi
   cf2da:	00 
   cf2db:	e8 70 32 04 00       	call   112550 <<alloc::vec::Vec<(hyperreal::computable::node::Computable, hyperreal::rational::arithmetic::Rational)> as core::clone::Clone>::clone>
   cf2e0:	49 8d 57 08          	lea    0x8(%r15),%rdx
   cf2e4:	4c 89 b4 24 30 01 00 	mov    %r14,0x130(%rsp)
   cf2eb:	00 
   cf2ec:	0f 10 44 24 70       	movups 0x70(%rsp),%xmm0
   cf2f1:	0f 29 84 24 00 01 00 	movaps %xmm0,0x100(%rsp)
   cf2f8:	00 
   cf2f9:	48 8b 84 24 80 00 00 	mov    0x80(%rsp),%rax
   cf300:	00 
   cf301:	48 89 84 24 10 01 00 	mov    %rax,0x110(%rsp)
   cf308:	00 
   cf309:	48 8b 84 24 58 01 00 	mov    0x158(%rsp),%rax
   cf310:	00 
   cf311:	48 8d 8c 24 18 01 00 	lea    0x118(%rsp),%rcx
   cf318:	00 
   cf319:	48 89 41 10          	mov    %rax,0x10(%rcx)
   cf31d:	0f 10 84 24 48 01 00 	movups 0x148(%rsp),%xmm0
   cf324:	00 
   cf325:	0f 11 01             	movups %xmm0,(%rcx)
   cf328:	48 8d bc 24 c8 00 00 	lea    0xc8(%rsp),%rdi
   cf32f:	00 
   cf330:	48 8d b4 24 00 01 00 	lea    0x100(%rsp),%rsi
   cf337:	00 
   cf338:	e8 13 dd ff ff       	call   cd050 <<hyperreal::computable::node::RadicalCoefficient>::multiply>
   cf33d:	48 83 bc 24 c8 00 00 	cmpq   $0xffffffffffffffff,0xc8(%rsp)
   cf344:	00 ff 
   cf346:	0f 84 70 02 00 00    	je     cf5bc <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x68c>
   cf34c:	48 8b 84 24 f8 00 00 	mov    0xf8(%rsp),%rax
   cf353:	00 
   cf354:	48 89 84 24 90 01 00 	mov    %rax,0x190(%rsp)
   cf35b:	00 
   cf35c:	0f 10 84 24 c8 00 00 	movups 0xc8(%rsp),%xmm0
   cf363:	00 
   cf364:	0f 10 8c 24 d8 00 00 	movups 0xd8(%rsp),%xmm1
   cf36b:	00 
   cf36c:	0f 10 94 24 e8 00 00 	movups 0xe8(%rsp),%xmm2
   cf373:	00 
   cf374:	0f 29 94 24 80 01 00 	movaps %xmm2,0x180(%rsp)
   cf37b:	00 
   cf37c:	0f 29 8c 24 70 01 00 	movaps %xmm1,0x170(%rsp)
   cf383:	00 
   cf384:	0f 29 84 24 60 01 00 	movaps %xmm0,0x160(%rsp)
   cf38b:	00 
   cf38c:	48 8d 7c 24 40       	lea    0x40(%rsp),%rdi
   cf391:	44 89 ee             	mov    %r13d,%esi
   cf394:	48 8d 94 24 60 01 00 	lea    0x160(%rsp),%rdx
   cf39b:	00 
   cf39c:	e8 ff ef ff ff       	call   ce3a0 <<hyperreal::computable::node::PiLaurentPolynomial>::insert>
   cf3a1:	84 c0                	test   %al,%al
   cf3a3:	0f 84 13 02 00 00    	je     cf5bc <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x68c>
   cf3a9:	49 83 c7 40          	add    $0x40,%r15
   cf3ad:	4c 3b 7c 24 60       	cmp    0x60(%rsp),%r15
   cf3b2:	0f 85 78 fe ff ff    	jne    cf230 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x300>
   cf3b8:	e9 43 01 00 00       	jmp    cf500 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x5d0>
   cf3bd:	4c 8b 2c 24          	mov    (%rsp),%r13
   cf3c1:	66 66 66 66 66 66 2e 	data16 data16 data16 data16 data16 cs nopw 0x0(%rax,%rax,1)
   cf3c8:	0f 1f 84 00 00 00 00 
   cf3cf:	00 
   cf3d0:	41 0f b7 5d 00       	movzwl 0x0(%r13),%ebx
   cf3d5:	66 03 5c 24 38       	add    0x38(%rsp),%bx
   cf3da:	0f 80 dc 01 00 00    	jo     cf5bc <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x68c>
   cf3e0:	f0 49 ff 06          	lock incq (%r14)
   cf3e4:	0f 8e b3 01 00 00    	jle    cf59d <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x66d>
   cf3ea:	4c 89 74 24 68       	mov    %r14,0x68(%rsp)
   cf3ef:	48 c7 44 24 70 00 00 	movq   $0x0,0x70(%rsp)
   cf3f6:	00 00 
   cf3f8:	48 c7 44 24 78 08 00 	movq   $0x8,0x78(%rsp)
   cf3ff:	00 00 
   cf401:	48 c7 84 24 80 00 00 	movq   $0x0,0x80(%rsp)
   cf408:	00 00 00 00 00 
   cf40d:	48 8d bc 24 48 01 00 	lea    0x148(%rsp),%rdi
   cf414:	00 
   cf415:	48 8d b4 24 a8 00 00 	lea    0xa8(%rsp),%rsi
   cf41c:	00 
   cf41d:	e8 2e 31 04 00       	call   112550 <<alloc::vec::Vec<(hyperreal::computable::node::Computable, hyperreal::rational::arithmetic::Rational)> as core::clone::Clone>::clone>
   cf422:	4c 89 b4 24 30 01 00 	mov    %r14,0x130(%rsp)
   cf429:	00 
   cf42a:	0f 10 44 24 70       	movups 0x70(%rsp),%xmm0
   cf42f:	0f 29 84 24 00 01 00 	movaps %xmm0,0x100(%rsp)
   cf436:	00 
   cf437:	48 8b 84 24 80 00 00 	mov    0x80(%rsp),%rax
   cf43e:	00 
   cf43f:	48 89 84 24 10 01 00 	mov    %rax,0x110(%rsp)
   cf446:	00 
   cf447:	48 8b 84 24 58 01 00 	mov    0x158(%rsp),%rax
   cf44e:	00 
   cf44f:	48 8d 8c 24 18 01 00 	lea    0x118(%rsp),%rcx
   cf456:	00 
   cf457:	48 89 41 10          	mov    %rax,0x10(%rcx)
   cf45b:	0f 10 84 24 48 01 00 	movups 0x148(%rsp),%xmm0
   cf462:	00 
   cf463:	0f 11 01             	movups %xmm0,(%rcx)
   cf466:	49 83 c5 08          	add    $0x8,%r13
   cf46a:	48 8d bc 24 c8 00 00 	lea    0xc8(%rsp),%rdi
   cf471:	00 
   cf472:	48 8d b4 24 00 01 00 	lea    0x100(%rsp),%rsi
   cf479:	00 
   cf47a:	4c 89 ea             	mov    %r13,%rdx
   cf47d:	e8 ce db ff ff       	call   cd050 <<hyperreal::computable::node::RadicalCoefficient>::multiply>
   cf482:	48 83 bc 24 c8 00 00 	cmpq   $0xffffffffffffffff,0xc8(%rsp)
   cf489:	00 ff 
   cf48b:	0f 84 2b 01 00 00    	je     cf5bc <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x68c>
   cf491:	48 8b 84 24 f8 00 00 	mov    0xf8(%rsp),%rax
   cf498:	00 
   cf499:	48 89 84 24 90 01 00 	mov    %rax,0x190(%rsp)
   cf4a0:	00 
   cf4a1:	0f 10 84 24 c8 00 00 	movups 0xc8(%rsp),%xmm0
   cf4a8:	00 
   cf4a9:	0f 10 8c 24 d8 00 00 	movups 0xd8(%rsp),%xmm1
   cf4b0:	00 
   cf4b1:	0f 10 94 24 e8 00 00 	movups 0xe8(%rsp),%xmm2
   cf4b8:	00 
   cf4b9:	0f 29 94 24 80 01 00 	movaps %xmm2,0x180(%rsp)
   cf4c0:	00 
   cf4c1:	0f 29 8c 24 70 01 00 	movaps %xmm1,0x170(%rsp)
   cf4c8:	00 
   cf4c9:	0f 29 84 24 60 01 00 	movaps %xmm0,0x160(%rsp)
   cf4d0:	00 
   cf4d1:	48 8d 7c 24 40       	lea    0x40(%rsp),%rdi
   cf4d6:	89 de                	mov    %ebx,%esi
   cf4d8:	48 8d 94 24 60 01 00 	lea    0x160(%rsp),%rdx
   cf4df:	00 
   cf4e0:	e8 bb ee ff ff       	call   ce3a0 <<hyperreal::computable::node::PiLaurentPolynomial>::insert>
   cf4e5:	84 c0                	test   %al,%al
   cf4e7:	0f 84 cf 00 00 00    	je     cf5bc <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x68c>
   cf4ed:	49 83 c5 38          	add    $0x38,%r13
   cf4f1:	4c 3b 6c 24 60       	cmp    0x60(%rsp),%r13
   cf4f6:	0f 85 d4 fe ff ff    	jne    cf3d0 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x4a0>
   cf4fc:	0f 1f 40 00          	nopl   0x0(%rax)
   cf500:	48 8d bc 24 90 00 00 	lea    0x90(%rsp),%rdi
   cf507:	00 
   cf508:	e8 93 17 fe ff       	call   b0ca0 <core::ptr::drop_glue::<hyperreal::computable::node::RadicalCoefficient>>
   cf50d:	48 8b 4c 24 08       	mov    0x8(%rsp),%rcx
   cf512:	48 89 c8             	mov    %rcx,%rax
   cf515:	48 3b 8c 24 88 00 00 	cmp    0x88(%rsp),%rcx
   cf51c:	00 
   cf51d:	4c 8b 74 24 10       	mov    0x10(%rsp),%r14
   cf522:	48 8d b4 24 98 00 00 	lea    0x98(%rsp),%rsi
   cf529:	00 
   cf52a:	0f 85 59 fc ff ff    	jne    cf189 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x259>
   cf530:	48 8b 94 24 88 00 00 	mov    0x88(%rsp),%rdx
   cf537:	00 
   cf538:	48 89 54 24 20       	mov    %rdx,0x20(%rsp)
   cf53d:	48 8d 7c 24 18       	lea    0x18(%rsp),%rdi
   cf542:	e8 19 dd 03 00       	call   10d260 <<alloc::vec::into_iter::IntoIter<(i16, hyperreal::computable::node::RadicalCoefficient)> as core::ops::drop::Drop>::drop>
   cf547:	48 8b 44 24 50       	mov    0x50(%rsp),%rax
   cf54c:	49 89 46 10          	mov    %rax,0x10(%r14)
   cf550:	0f 10 44 24 40       	movups 0x40(%rsp),%xmm0
   cf555:	41 0f 11 06          	movups %xmm0,(%r14)
   cf559:	49 ff c4             	inc    %r12
   cf55c:	48 8b 3c 24          	mov    (%rsp),%rdi
   cf560:	49 83 fc 01          	cmp    $0x1,%r12
   cf564:	0f 84 ed 00 00 00    	je     cf657 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x727>
   cf56a:	48 8d 5f 40          	lea    0x40(%rdi),%rbx
   cf56e:	49 ff cc             	dec    %r12
   cf571:	e8 5a 1d 04 00       	call   1112d0 <core::ptr::drop_glue::<(i16, hyperreal::computable::node::RadicalCoefficient)>>
   cf576:	48 89 df             	mov    %rbx,%rdi
   cf579:	eb e5                	jmp    cf560 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x630>
   cf57b:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
   cf580:	48 89 44 24 20       	mov    %rax,0x20(%rsp)
   cf585:	48 8b 04 24          	mov    (%rsp),%rax
   cf589:	0f b7 4c 24 38       	movzwl 0x38(%rsp),%ecx
   cf58e:	66 03 08             	add    (%rax),%cx
   cf591:	70 33                	jo     cf5c6 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x696>
   cf593:	f0 49 ff 06          	lock incq (%r14)
   cf597:	0f 8f ec 00 00 00    	jg     cf689 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x759>
   cf59d:	0f 0b                	ud2
   cf59f:	48 89 5c 24 20       	mov    %rbx,0x20(%rsp)
   cf5a4:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
   cf5a9:	48 c7 00 ff ff ff ff 	movq   $0xffffffffffffffff,(%rax)
   cf5b0:	48 8d 7c 24 18       	lea    0x18(%rsp),%rdi
   cf5b5:	e8 a6 dc 03 00       	call   10d260 <<alloc::vec::into_iter::IntoIter<(i16, hyperreal::computable::node::RadicalCoefficient)> as core::ops::drop::Drop>::drop>
   cf5ba:	eb 2d                	jmp    cf5e9 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x6b9>
   cf5bc:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
   cf5c1:	48 89 44 24 20       	mov    %rax,0x20(%rsp)
   cf5c6:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
   cf5cb:	48 c7 00 ff ff ff ff 	movq   $0xffffffffffffffff,(%rax)
   cf5d2:	48 8d bc 24 90 00 00 	lea    0x90(%rsp),%rdi
   cf5d9:	00 
   cf5da:	e8 c1 16 fe ff       	call   b0ca0 <core::ptr::drop_glue::<hyperreal::computable::node::RadicalCoefficient>>
   cf5df:	48 8d 7c 24 18       	lea    0x18(%rsp),%rdi
   cf5e4:	e8 77 dc 03 00       	call   10d260 <<alloc::vec::into_iter::IntoIter<(i16, hyperreal::computable::node::RadicalCoefficient)> as core::ops::drop::Drop>::drop>
   cf5e9:	48 8b 7c 24 48       	mov    0x48(%rsp),%rdi
   cf5ee:	4c 8b 74 24 50       	mov    0x50(%rsp),%r14
   cf5f3:	49 ff c6             	inc    %r14
   cf5f6:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
   cf5fd:	00 00 00 
   cf600:	49 83 fe 01          	cmp    $0x1,%r14
   cf604:	74 11                	je     cf617 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x6e7>
   cf606:	48 8d 5f 40          	lea    0x40(%rdi),%rbx
   cf60a:	49 ff ce             	dec    %r14
   cf60d:	e8 be 1c 04 00       	call   1112d0 <core::ptr::drop_glue::<(i16, hyperreal::computable::node::RadicalCoefficient)>>
   cf612:	48 89 df             	mov    %rbx,%rdi
   cf615:	eb e9                	jmp    cf600 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x6d0>
   cf617:	48 8b 74 24 40       	mov    0x40(%rsp),%rsi
   cf61c:	48 85 f6             	test   %rsi,%rsi
   cf61f:	74 14                	je     cf635 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x705>
   cf621:	48 8b 7c 24 48       	mov    0x48(%rsp),%rdi
   cf626:	48 c1 e6 06          	shl    $0x6,%rsi
   cf62a:	ba 08 00 00 00       	mov    $0x8,%edx
   cf62f:	ff 15 1b f5 09 00    	call   *0x9f51b(%rip)        # 16eb50 <_DYNAMIC+0x2c0>
   cf635:	49 ff c4             	inc    %r12
   cf638:	48 8b 3c 24          	mov    (%rsp),%rdi
   cf63c:	0f 1f 40 00          	nopl   0x0(%rax)
   cf640:	49 83 fc 01          	cmp    $0x1,%r12
   cf644:	74 11                	je     cf657 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x727>
   cf646:	48 8d 5f 40          	lea    0x40(%rdi),%rbx
   cf64a:	49 ff cc             	dec    %r12
   cf64d:	e8 7e 1c 04 00       	call   1112d0 <core::ptr::drop_glue::<(i16, hyperreal::computable::node::RadicalCoefficient)>>
   cf652:	48 89 df             	mov    %rbx,%rdi
   cf655:	eb e9                	jmp    cf640 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x710>
   cf657:	48 8b 44 24 58       	mov    0x58(%rsp),%rax
   cf65c:	48 8b 30             	mov    (%rax),%rsi
   cf65f:	48 85 f6             	test   %rsi,%rsi
   cf662:	74 13                	je     cf677 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x747>
   cf664:	48 c1 e6 06          	shl    $0x6,%rsi
   cf668:	ba 08 00 00 00       	mov    $0x8,%edx
   cf66d:	48 8b 3c 24          	mov    (%rsp),%rdi
   cf671:	ff 15 d9 f4 09 00    	call   *0x9f4d9(%rip)        # 16eb50 <_DYNAMIC+0x2c0>
   cf677:	48 81 c4 d8 01 00 00 	add    $0x1d8,%rsp
   cf67e:	5b                   	pop    %rbx
   cf67f:	41 5c                	pop    %r12
   cf681:	41 5d                	pop    %r13
   cf683:	41 5e                	pop    %r14
   cf685:	41 5f                	pop    %r15
   cf687:	5d                   	pop    %rbp
   cf688:	c3                   	ret
   cf689:	4c 89 74 24 68       	mov    %r14,0x68(%rsp)
   cf68e:	31 ff                	xor    %edi,%edi
   cf690:	48 89 ee             	mov    %rbp,%rsi
   cf693:	ff 15 27 f5 09 00    	call   *0x9f527(%rip)        # 16ebc0 <_DYNAMIC+0x330>
   cf699:	e9 ff fe ff ff       	jmp    cf59d <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x66d>
   cf69e:	48 89 5c 24 20       	mov    %rbx,0x20(%rsp)
   cf6a3:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
   cf6a8:	48 c7 00 ff ff ff ff 	movq   $0xffffffffffffffff,(%rax)
   cf6af:	48 8d bc 24 90 00 00 	lea    0x90(%rsp),%rdi
   cf6b6:	00 
   cf6b7:	e8 e4 15 fe ff       	call   b0ca0 <core::ptr::drop_glue::<hyperreal::computable::node::RadicalCoefficient>>
   cf6bc:	e9 ef fe ff ff       	jmp    cf5b0 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x680>
   cf6c1:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
   cf6c6:	48 89 44 24 20       	mov    %rax,0x20(%rsp)
   cf6cb:	bf 08 00 00 00       	mov    $0x8,%edi
   cf6d0:	eb be                	jmp    cf690 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x760>
   cf6d2:	49 89 c7             	mov    %rax,%r15
   cf6d5:	eb 5c                	jmp    cf733 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x803>
   cf6d7:	49 89 c7             	mov    %rax,%r15
   cf6da:	e9 56 01 00 00       	jmp    cf835 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x905>
   cf6df:	49 89 c7             	mov    %rax,%r15
   cf6e2:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
   cf6e7:	48 89 44 24 20       	mov    %rax,0x20(%rsp)
   cf6ec:	e9 44 01 00 00       	jmp    cf835 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x905>
   cf6f1:	eb 08                	jmp    cf6fb <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x7cb>
   cf6f3:	49 89 c7             	mov    %rax,%r15
   cf6f6:	e9 44 01 00 00       	jmp    cf83f <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x90f>
   cf6fb:	49 89 c7             	mov    %rax,%r15
   cf6fe:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
   cf703:	48 89 44 24 20       	mov    %rax,0x20(%rsp)
   cf708:	48 8d 7c 24 70       	lea    0x70(%rsp),%rdi
   cf70d:	e8 5e 02 fe ff       	call   af970 <core::ptr::drop_glue::<alloc::vec::Vec<(hyperreal::rational::arithmetic::Rational, hyperreal::rational::arithmetic::Rational)>>>
   cf712:	e9 00 01 00 00       	jmp    cf817 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x8e7>
   cf717:	eb 00                	jmp    cf719 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x7e9>
   cf719:	49 89 c7             	mov    %rax,%r15
   cf71c:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
   cf721:	48 89 44 24 20       	mov    %rax,0x20(%rsp)
   cf726:	e9 fd 00 00 00       	jmp    cf828 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x8f8>
   cf72b:	49 89 c7             	mov    %rax,%r15
   cf72e:	48 89 5c 24 20       	mov    %rbx,0x20(%rsp)
   cf733:	48 8d 7c 24 18       	lea    0x18(%rsp),%rdi
   cf738:	e8 23 db 03 00       	call   10d260 <<alloc::vec::into_iter::IntoIter<(i16, hyperreal::computable::node::RadicalCoefficient)> as core::ops::drop::Drop>::drop>
   cf73d:	e9 fd 00 00 00       	jmp    cf83f <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x90f>
   cf742:	49 89 c7             	mov    %rax,%r15
   cf745:	66 66 2e 0f 1f 84 00 	data16 cs nopw 0x0(%rax,%rax,1)
   cf74c:	00 00 00 00 
   cf750:	49 ff cc             	dec    %r12
   cf753:	0f 84 8d 00 00 00    	je     cf7e6 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x8b6>
   cf759:	4c 8d 73 40          	lea    0x40(%rbx),%r14
   cf75d:	48 89 df             	mov    %rbx,%rdi
   cf760:	e8 6b 1b 04 00       	call   1112d0 <core::ptr::drop_glue::<(i16, hyperreal::computable::node::RadicalCoefficient)>>
   cf765:	4c 89 f3             	mov    %r14,%rbx
   cf768:	eb e6                	jmp    cf750 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x820>
   cf76a:	ff 15 c8 f3 09 00    	call   *0x9f3c8(%rip)        # 16eb38 <_DYNAMIC+0x2a8>
   cf770:	49 89 c7             	mov    %rax,%r15
   cf773:	66 66 66 66 2e 0f 1f 	data16 data16 data16 cs nopw 0x0(%rax,%rax,1)
   cf77a:	84 00 00 00 00 00 
   cf780:	49 ff ce             	dec    %r14
   cf783:	74 11                	je     cf796 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x866>
   cf785:	4c 8d 63 40          	lea    0x40(%rbx),%r12
   cf789:	48 89 df             	mov    %rbx,%rdi
   cf78c:	e8 3f 1b 04 00       	call   1112d0 <core::ptr::drop_glue::<(i16, hyperreal::computable::node::RadicalCoefficient)>>
   cf791:	4c 89 e3             	mov    %r12,%rbx
   cf794:	eb ea                	jmp    cf780 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x850>
   cf796:	48 8b 74 24 40       	mov    0x40(%rsp),%rsi
   cf79b:	48 85 f6             	test   %rsi,%rsi
   cf79e:	0f 84 a5 00 00 00    	je     cf849 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x919>
   cf7a4:	48 8b 7c 24 48       	mov    0x48(%rsp),%rdi
   cf7a9:	48 c1 e6 06          	shl    $0x6,%rsi
   cf7ad:	ba 08 00 00 00       	mov    $0x8,%edx
   cf7b2:	ff 15 98 f3 09 00    	call   *0x9f398(%rip)        # 16eb50 <_DYNAMIC+0x2c0>
   cf7b8:	e9 8c 00 00 00       	jmp    cf849 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x919>
   cf7bd:	ff 15 75 f3 09 00    	call   *0x9f375(%rip)        # 16eb38 <_DYNAMIC+0x2a8>
   cf7c3:	49 89 c7             	mov    %rax,%r15
   cf7c6:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
   cf7cd:	00 00 00 
   cf7d0:	49 ff cc             	dec    %r12
   cf7d3:	74 11                	je     cf7e6 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x8b6>
   cf7d5:	4c 8d 73 40          	lea    0x40(%rbx),%r14
   cf7d9:	48 89 df             	mov    %rbx,%rdi
   cf7dc:	e8 ef 1a 04 00       	call   1112d0 <core::ptr::drop_glue::<(i16, hyperreal::computable::node::RadicalCoefficient)>>
   cf7e1:	4c 89 f3             	mov    %r14,%rbx
   cf7e4:	eb ea                	jmp    cf7d0 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x8a0>
   cf7e6:	48 8b 44 24 58       	mov    0x58(%rsp),%rax
   cf7eb:	48 8b 30             	mov    (%rax),%rsi
   cf7ee:	48 85 f6             	test   %rsi,%rsi
   cf7f1:	74 60                	je     cf853 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x923>
   cf7f3:	48 c1 e6 06          	shl    $0x6,%rsi
   cf7f7:	ba 08 00 00 00       	mov    $0x8,%edx
   cf7fc:	48 8b 3c 24          	mov    (%rsp),%rdi
   cf800:	ff 15 4a f3 09 00    	call   *0x9f34a(%rip)        # 16eb50 <_DYNAMIC+0x2c0>
   cf806:	4c 89 ff             	mov    %r15,%rdi
   cf809:	e8 22 90 09 00       	call   168830 <_Unwind_Resume@plt>
   cf80e:	ff 15 24 f3 09 00    	call   *0x9f324(%rip)        # 16eb38 <_DYNAMIC+0x2a8>
   cf814:	49 89 c7             	mov    %rax,%r15
   cf817:	f0 49 ff 0e          	lock decq (%r14)
   cf81b:	75 0b                	jne    cf828 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x8f8>
   cf81d:	48 8d 7c 24 68       	lea    0x68(%rsp),%rdi
   cf822:	ff 15 08 f3 09 00    	call   *0x9f308(%rip)        # 16eb30 <_DYNAMIC+0x2a0>
   cf828:	48 8d bc 24 90 00 00 	lea    0x90(%rsp),%rdi
   cf82f:	00 
   cf830:	e8 6b 14 fe ff       	call   b0ca0 <core::ptr::drop_glue::<hyperreal::computable::node::RadicalCoefficient>>
   cf835:	48 8d 7c 24 18       	lea    0x18(%rsp),%rdi
   cf83a:	e8 21 da 03 00       	call   10d260 <<alloc::vec::into_iter::IntoIter<(i16, hyperreal::computable::node::RadicalCoefficient)> as core::ops::drop::Drop>::drop>
   cf83f:	48 8d 7c 24 40       	lea    0x40(%rsp),%rdi
   cf844:	e8 f7 01 fe ff       	call   afa40 <core::ptr::drop_glue::<alloc::vec::Vec<(i16, hyperreal::computable::node::RadicalCoefficient)>>>
   cf849:	48 8b 7c 24 58       	mov    0x58(%rsp),%rdi
   cf84e:	e8 ed 01 fe ff       	call   afa40 <core::ptr::drop_glue::<alloc::vec::Vec<(i16, hyperreal::computable::node::RadicalCoefficient)>>>
   cf853:	4c 89 ff             	mov    %r15,%rdi
   cf856:	e8 d5 8f 09 00       	call   168830 <_Unwind_Resume@plt>
   cf85b:	ff 15 d7 f2 09 00    	call   *0x9f2d7(%rip)        # 16eb38 <_DYNAMIC+0x2a8>
   cf861:	ff 15 d1 f2 09 00    	call   *0x9f2d1(%rip)        # 16eb38 <_DYNAMIC+0x2a8>
