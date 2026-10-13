
/home/tim/Documents/GitHub/workspace/csgrs/target/outlined-same-radicand-products/cost-v2-binaries/direct-quadratic-time:     file format elf64-x86-64


Disassembly of section .text:

00000000000cd100 <<hyperreal::computable::node::RadicalCoefficient>::multiply>:
   cd100:	55                   	push   %rbp
   cd101:	41 57                	push   %r15
   cd103:	41 56                	push   %r14
   cd105:	41 55                	push   %r13
   cd107:	41 54                	push   %r12
   cd109:	53                   	push   %rbx
   cd10a:	48 81 ec 08 01 00 00 	sub    $0x108,%rsp
   cd111:	4c 8d 7e 18          	lea    0x18(%rsi),%r15
   cd115:	48 89 74 24 08       	mov    %rsi,0x8(%rsp)
   cd11a:	48 83 7e 28 00       	cmpq   $0x0,0x28(%rsi)
   cd11f:	74 42                	je     cd163 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x63>
   cd121:	48 83 7a 10 00       	cmpq   $0x0,0x10(%rdx)
   cd126:	75 54                	jne    cd17c <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x7c>
   cd128:	48 83 7a 28 00       	cmpq   $0x0,0x28(%rdx)
   cd12d:	75 4d                	jne    cd17c <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x7c>
   cd12f:	48 89 7c 24 40       	mov    %rdi,0x40(%rsp)
   cd134:	4c 89 7c 24 58       	mov    %r15,0x58(%rsp)
   cd139:	48 8b 7c 24 08       	mov    0x8(%rsp),%rdi
   cd13e:	48 8d 6f 10          	lea    0x10(%rdi),%rbp
   cd142:	4c 8d 6f 08          	lea    0x8(%rdi),%r13
   cd146:	48 8b 42 08          	mov    0x8(%rdx),%rax
   cd14a:	48 89 44 24 60       	mov    %rax,0x60(%rsp)
   cd14f:	b0 01                	mov    $0x1,%al
   cd151:	89 44 24 2c          	mov    %eax,0x2c(%rsp)
   cd155:	48 c7 44 24 18 00 00 	movq   $0x0,0x18(%rsp)
   cd15c:	00 00 
   cd15e:	e9 0e 02 00 00       	jmp    cd371 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x271>
   cd163:	48 8b 42 28          	mov    0x28(%rdx),%rax
   cd167:	48 85 c0             	test   %rax,%rax
   cd16a:	74 39                	je     cd1a5 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xa5>
   cd16c:	48 8b 4c 24 08       	mov    0x8(%rsp),%rcx
   cd171:	48 83 79 10 00       	cmpq   $0x0,0x10(%rcx)
   cd176:	0f 84 af 01 00 00    	je     cd32b <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x22b>
   cd17c:	48 c7 07 ff ff ff ff 	movq   $0xffffffffffffffff,(%rdi)
   cd183:	48 8b 5c 24 08       	mov    0x8(%rsp),%rbx
   cd188:	48 8b 43 30          	mov    0x30(%rbx),%rax
   cd18c:	f0 48 ff 08          	lock decq (%rax)
   cd190:	0f 85 5d 03 00 00    	jne    cd4f3 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x3f3>
   cd196:	48 8d 7b 30          	lea    0x30(%rbx),%rdi
   cd19a:	ff 15 10 21 0a 00    	call   *0xa2110(%rip)        # 16f2b0 <_DYNAMIC+0x2a0>
   cd1a0:	e9 4e 03 00 00       	jmp    cd4f3 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x3f3>
   cd1a5:	48 89 7c 24 40       	mov    %rdi,0x40(%rsp)
   cd1aa:	4c 89 7c 24 58       	mov    %r15,0x58(%rsp)
   cd1af:	48 8b 72 08          	mov    0x8(%rdx),%rsi
   cd1b3:	48 8b 4a 10          	mov    0x10(%rdx),%rcx
   cd1b7:	48 8b 7c 24 08       	mov    0x8(%rsp),%rdi
   cd1bc:	48 8d 6f 10          	lea    0x10(%rdi),%rbp
   cd1c0:	48 8b 47 10          	mov    0x10(%rdi),%rax
   cd1c4:	48 83 f0 01          	xor    $0x1,%rax
   cd1c8:	4c 8d 6f 08          	lea    0x8(%rdi),%r13
   cd1cc:	48 89 4c 24 18       	mov    %rcx,0x18(%rsp)
   cd1d1:	48 83 f1 01          	xor    $0x1,%rcx
   cd1d5:	48 09 c1             	or     %rax,%rcx
   cd1d8:	48 89 74 24 60       	mov    %rsi,0x60(%rsp)
   cd1dd:	0f 85 88 01 00 00    	jne    cd36b <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x26b>
   cd1e3:	49 89 d4             	mov    %rdx,%r12
   cd1e6:	48 8b 5f 08          	mov    0x8(%rdi),%rbx
   cd1ea:	41 b6 01             	mov    $0x1,%r14b
   cd1ed:	48 89 df             	mov    %rbx,%rdi
   cd1f0:	ff 15 62 20 0a 00    	call   *0xa2062(%rip)        # 16f258 <_DYNAMIC+0x248>
   cd1f6:	b9 01 00 00 00       	mov    $0x1,%ecx
   cd1fb:	48 89 4c 24 18       	mov    %rcx,0x18(%rsp)
   cd200:	84 c0                	test   %al,%al
   cd202:	0f 84 8e 05 00 00    	je     cd796 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x696>
   cd208:	4c 8b 3b             	mov    (%rbx),%r15
   cd20b:	41 8b 47 60          	mov    0x60(%r15),%eax
   cd20f:	84 c0                	test   %al,%al
   cd211:	0f 88 41 07 00 00    	js     cd958 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x858>
   cd217:	41 80 7f 64 02       	cmpb   $0x2,0x64(%r15)
   cd21c:	75 63                	jne    cd281 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x181>
   cd21e:	4c 8b 35 73 20 0a 00 	mov    0xa2073(%rip),%r14        # 16f298 <_DYNAMIC+0x288>
   cd225:	41 8b 46 18          	mov    0x18(%r14),%eax
   cd229:	85 c0                	test   %eax,%eax
   cd22b:	0f 85 bd 09 00 00    	jne    cdbee <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xaee>
   cd231:	49 8b 57 20          	mov    0x20(%r15),%rdx
   cd235:	49 3b 56 10          	cmp    0x10(%r14),%rdx
   cd239:	75 46                	jne    cd281 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x181>
   cd23b:	49 8b 76 08          	mov    0x8(%r14),%rsi
   cd23f:	49 8b 7f 18          	mov    0x18(%r15),%rdi
   cd243:	48 c1 e2 03          	shl    $0x3,%rdx
   cd247:	ff 15 53 20 0a 00    	call   *0xa2053(%rip)        # 16f2a0 <bcmp@GLIBC_2.2.5>
   cd24d:	85 c0                	test   %eax,%eax
   cd24f:	75 30                	jne    cd281 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x181>
   cd251:	41 8b 46 18          	mov    0x18(%r14),%eax
   cd255:	85 c0                	test   %eax,%eax
   cd257:	0f 85 e8 09 00 00    	jne    cdc45 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xb45>
   cd25d:	49 8b 57 38          	mov    0x38(%r15),%rdx
   cd261:	49 3b 56 10          	cmp    0x10(%r14),%rdx
   cd265:	75 1a                	jne    cd281 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x181>
   cd267:	49 8b 76 08          	mov    0x8(%r14),%rsi
   cd26b:	49 8b 7f 30          	mov    0x30(%r15),%rdi
   cd26f:	48 c1 e2 03          	shl    $0x3,%rdx
   cd273:	ff 15 27 20 0a 00    	call   *0xa2027(%rip)        # 16f2a0 <bcmp@GLIBC_2.2.5>
   cd279:	85 c0                	test   %eax,%eax
   cd27b:	0f 84 15 05 00 00    	je     cd796 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x696>
   cd281:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
   cd286:	48 8d 78 30          	lea    0x30(%rax),%rdi
   cd28a:	49 83 c4 30          	add    $0x30,%r12
   cd28e:	41 b6 01             	mov    $0x1,%r14b
   cd291:	48 89 7c 24 30       	mov    %rdi,0x30(%rsp)
   cd296:	4d 89 e7             	mov    %r12,%r15
   cd299:	4c 89 e6             	mov    %r12,%rsi
   cd29c:	e8 af 44 fc ff       	call   91750 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cd2a1:	48 8d 7b 08          	lea    0x8(%rbx),%rdi
   cd2a5:	48 89 44 24 48       	mov    %rax,0x48(%rsp)
   cd2aa:	48 89 44 24 68       	mov    %rax,0x68(%rsp)
   cd2af:	48 89 bc 24 90 00 00 	mov    %rdi,0x90(%rsp)
   cd2b6:	00 
   cd2b7:	4c 89 fe             	mov    %r15,%rsi
   cd2ba:	e8 91 44 fc ff       	call   91750 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cd2bf:	49 89 c4             	mov    %rax,%r12
   cd2c2:	48 8b 44 24 60       	mov    0x60(%rsp),%rax
   cd2c7:	48 8d 78 08          	lea    0x8(%rax),%rdi
   cd2cb:	4c 89 64 24 50       	mov    %r12,0x50(%rsp)
   cd2d0:	b0 01                	mov    $0x1,%al
   cd2d2:	89 44 24 10          	mov    %eax,0x10(%rsp)
   cd2d6:	41 b7 01             	mov    $0x1,%r15b
   cd2d9:	49 89 fe             	mov    %rdi,%r14
   cd2dc:	48 8b 74 24 30       	mov    0x30(%rsp),%rsi
   cd2e1:	e8 6a 44 fc ff       	call   91750 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cd2e6:	48 89 44 24 38       	mov    %rax,0x38(%rsp)
   cd2eb:	48 89 c6             	mov    %rax,%rsi
   cd2ee:	4c 89 e0             	mov    %r12,%rax
   cd2f1:	45 0f b6 64 24 64    	movzbl 0x64(%r12),%r12d
   cd2f7:	41 80 fc 01          	cmp    $0x1,%r12b
   cd2fb:	48 89 74 24 18       	mov    %rsi,0x18(%rsp)
   cd300:	49 89 df             	mov    %rbx,%r15
   cd303:	0f 85 8a 06 00 00    	jne    cd993 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x893>
   cd309:	f0 48 ff 08          	lock decq (%rax)
   cd30d:	48 89 f0             	mov    %rsi,%rax
   cd310:	0f 85 a4 06 00 00    	jne    cd9ba <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x8ba>
   cd316:	48 8d 7c 24 50       	lea    0x50(%rsp),%rdi
   cd31b:	ff 15 8f 1f 0a 00    	call   *0xa1f8f(%rip)        # 16f2b0 <_DYNAMIC+0x2a0>
   cd321:	48 8b 44 24 18       	mov    0x18(%rsp),%rax
   cd326:	e9 8f 06 00 00       	jmp    cd9ba <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x8ba>
   cd32b:	48 89 7c 24 40       	mov    %rdi,0x40(%rsp)
   cd330:	4c 89 7c 24 58       	mov    %r15,0x58(%rsp)
   cd335:	48 8b 7c 24 08       	mov    0x8(%rsp),%rdi
   cd33a:	48 8d 6f 10          	lea    0x10(%rdi),%rbp
   cd33e:	4c 8d 6f 08          	lea    0x8(%rdi),%r13
   cd342:	48 8b 4a 08          	mov    0x8(%rdx),%rcx
   cd346:	48 89 4c 24 60       	mov    %rcx,0x60(%rsp)
   cd34b:	48 8b 4a 10          	mov    0x10(%rdx),%rcx
   cd34f:	48 89 4c 24 18       	mov    %rcx,0x18(%rsp)
   cd354:	48 c1 e0 03          	shl    $0x3,%rax
   cd358:	48 8d 04 40          	lea    (%rax,%rax,2),%rax
   cd35c:	48 89 44 24 30       	mov    %rax,0x30(%rsp)
   cd361:	c7 44 24 2c 00 00 00 	movl   $0x0,0x2c(%rsp)
   cd368:	00 
   cd369:	eb 0f                	jmp    cd37a <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x27a>
   cd36b:	b0 01                	mov    $0x1,%al
   cd36d:	89 44 24 2c          	mov    %eax,0x2c(%rsp)
   cd371:	48 c7 44 24 30 00 00 	movq   $0x0,0x30(%rsp)
   cd378:	00 00 
   cd37a:	48 83 c7 30          	add    $0x30,%rdi
   cd37e:	48 89 94 24 f0 00 00 	mov    %rdx,0xf0(%rsp)
   cd385:	00 
   cd386:	48 8d 72 30          	lea    0x30(%rdx),%rsi
   cd38a:	41 b6 01             	mov    $0x1,%r14b
   cd38d:	48 89 7c 24 48       	mov    %rdi,0x48(%rsp)
   cd392:	48 89 74 24 20       	mov    %rsi,0x20(%rsp)
   cd397:	e8 b4 43 fc ff       	call   91750 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cd39c:	48 89 84 24 d8 00 00 	mov    %rax,0xd8(%rsp)
   cd3a3:	00 
   cd3a4:	48 c7 84 24 a8 00 00 	movq   $0x0,0xa8(%rsp)
   cd3ab:	00 00 00 00 00 
   cd3b0:	48 c7 84 24 b0 00 00 	movq   $0x8,0xb0(%rsp)
   cd3b7:	00 08 00 00 00 
   cd3bc:	0f 57 c0             	xorps  %xmm0,%xmm0
   cd3bf:	0f 11 84 24 b8 00 00 	movups %xmm0,0xb8(%rsp)
   cd3c6:	00 
   cd3c7:	48 c7 84 24 c8 00 00 	movq   $0x8,0xc8(%rsp)
   cd3ce:	00 08 00 00 00 
   cd3d3:	48 c7 84 24 d0 00 00 	movq   $0x0,0xd0(%rsp)
   cd3da:	00 00 00 00 00 
   cd3df:	4d 8b 7d 00          	mov    0x0(%r13),%r15
   cd3e3:	4c 8b 65 00          	mov    0x0(%rbp),%r12
   cd3e7:	49 c1 e4 04          	shl    $0x4,%r12
   cd3eb:	48 8d 9c 24 a8 00 00 	lea    0xa8(%rsp),%rbx
   cd3f2:	00 
   cd3f3:	4c 8b 6c 24 40       	mov    0x40(%rsp),%r13
   cd3f8:	0f 1f 84 00 00 00 00 	nopl   0x0(%rax,%rax,1)
   cd3ff:	00 
   cd400:	4d 85 e4             	test   %r12,%r12
   cd403:	74 3f                	je     cd444 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x344>
   cd405:	49 8b 07             	mov    (%r15),%rax
   cd408:	f0 48 ff 00          	lock incq (%rax)
   cd40c:	0f 8e ae 08 00 00    	jle    cdcc0 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xbc0>
   cd412:	49 8d 7f 08          	lea    0x8(%r15),%rdi
   cd416:	4d 8b 37             	mov    (%r15),%r14
   cd419:	4c 89 74 24 70       	mov    %r14,0x70(%rsp)
   cd41e:	48 8b 74 24 20       	mov    0x20(%rsp),%rsi
   cd423:	e8 28 43 fc ff       	call   91750 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cd428:	48 89 df             	mov    %rbx,%rdi
   cd42b:	4c 89 f6             	mov    %r14,%rsi
   cd42e:	48 89 c2             	mov    %rax,%rdx
   cd431:	e8 1a f4 ff ff       	call   cc850 <<hyperreal::computable::node::RadicalCoefficient>::insert_radical>
   cd436:	49 83 c7 10          	add    $0x10,%r15
   cd43a:	49 83 c4 f0          	add    $0xfffffffffffffff0,%r12
   cd43e:	84 c0                	test   %al,%al
   cd440:	75 be                	jne    cd400 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x300>
   cd442:	eb 75                	jmp    cd4b9 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x3b9>
   cd444:	4c 8b 7c 24 18       	mov    0x18(%rsp),%r15
   cd449:	49 c1 e7 04          	shl    $0x4,%r15
   cd44d:	48 8d 9c 24 a8 00 00 	lea    0xa8(%rsp),%rbx
   cd454:	00 
   cd455:	4c 89 bc 24 f8 00 00 	mov    %r15,0xf8(%rsp)
   cd45c:	00 
   cd45d:	4c 8b 64 24 60       	mov    0x60(%rsp),%r12
   cd462:	66 66 66 66 66 2e 0f 	data16 data16 data16 data16 cs nopw 0x0(%rax,%rax,1)
   cd469:	1f 84 00 00 00 00 00 
   cd470:	4d 85 ff             	test   %r15,%r15
   cd473:	0f 84 04 01 00 00    	je     cd57d <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x47d>
   cd479:	49 8b 04 24          	mov    (%r12),%rax
   cd47d:	f0 48 ff 00          	lock incq (%rax)
   cd481:	0f 8e 39 08 00 00    	jle    cdcc0 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xbc0>
   cd487:	49 8d 7c 24 08       	lea    0x8(%r12),%rdi
   cd48c:	4d 8b 34 24          	mov    (%r12),%r14
   cd490:	4c 89 74 24 70       	mov    %r14,0x70(%rsp)
   cd495:	48 8b 74 24 48       	mov    0x48(%rsp),%rsi
   cd49a:	e8 b1 42 fc ff       	call   91750 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cd49f:	48 89 df             	mov    %rbx,%rdi
   cd4a2:	4c 89 f6             	mov    %r14,%rsi
   cd4a5:	48 89 c2             	mov    %rax,%rdx
   cd4a8:	e8 a3 f3 ff ff       	call   cc850 <<hyperreal::computable::node::RadicalCoefficient>::insert_radical>
   cd4ad:	49 83 c4 10          	add    $0x10,%r12
   cd4b1:	49 83 c7 f0          	add    $0xfffffffffffffff0,%r15
   cd4b5:	84 c0                	test   %al,%al
   cd4b7:	75 b7                	jne    cd470 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x370>
   cd4b9:	49 c7 45 00 ff ff ff 	movq   $0xffffffffffffffff,0x0(%r13)
   cd4c0:	ff 
   cd4c1:	41 b6 01             	mov    $0x1,%r14b
   cd4c4:	48 8d bc 24 a8 00 00 	lea    0xa8(%rsp),%rdi
   cd4cb:	00 
   cd4cc:	e8 7f 38 fe ff       	call   b0d50 <core::ptr::drop_glue::<hyperreal::computable::node::RadicalCoefficient>>
   cd4d1:	48 8b 5c 24 08       	mov    0x8(%rsp),%rbx
   cd4d6:	48 8b 43 30          	mov    0x30(%rbx),%rax
   cd4da:	f0 48 ff 08          	lock decq (%rax)
   cd4de:	75 0a                	jne    cd4ea <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x3ea>
   cd4e0:	48 8d 7b 30          	lea    0x30(%rbx),%rdi
   cd4e4:	ff 15 c6 1d 0a 00    	call   *0xa1dc6(%rip)        # 16f2b0 <_DYNAMIC+0x2a0>
   cd4ea:	45 84 f6             	test   %r14b,%r14b
   cd4ed:	4c 8d 7b 18          	lea    0x18(%rbx),%r15
   cd4f1:	74 23                	je     cd516 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x416>
   cd4f3:	48 89 df             	mov    %rbx,%rdi
   cd4f6:	e8 35 5e 04 00       	call   113330 <<alloc::vec::Vec<(hyperreal::rational::arithmetic::Rational, hyperreal::rational::arithmetic::Rational)> as core::ops::drop::Drop>::drop>
   cd4fb:	48 8b 33             	mov    (%rbx),%rsi
   cd4fe:	48 85 f6             	test   %rsi,%rsi
   cd501:	74 13                	je     cd516 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x416>
   cd503:	48 8b 7b 08          	mov    0x8(%rbx),%rdi
   cd507:	48 c1 e6 04          	shl    $0x4,%rsi
   cd50b:	ba 08 00 00 00       	mov    $0x8,%edx
   cd510:	ff 15 ba 1d 0a 00    	call   *0xa1dba(%rip)        # 16f2d0 <_DYNAMIC+0x2c0>
   cd516:	48 8b 7b 20          	mov    0x20(%rbx),%rdi
   cd51a:	4c 8b 73 28          	mov    0x28(%rbx),%r14
   cd51e:	49 ff c6             	inc    %r14
   cd521:	66 66 66 66 66 66 2e 	data16 data16 data16 data16 data16 cs nopw 0x0(%rax,%rax,1)
   cd528:	0f 1f 84 00 00 00 00 
   cd52f:	00 
   cd530:	49 83 fe 01          	cmp    $0x1,%r14
   cd534:	74 11                	je     cd547 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x447>
   cd536:	48 8d 5f 18          	lea    0x18(%rdi),%rbx
   cd53a:	49 ff ce             	dec    %r14
   cd53d:	e8 7e 42 04 00       	call   1117c0 <core::ptr::drop_glue::<(hyperreal::computable::node::Computable, hyperreal::rational::arithmetic::Rational)>>
   cd542:	48 89 df             	mov    %rbx,%rdi
   cd545:	eb e9                	jmp    cd530 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x430>
   cd547:	49 8b 07             	mov    (%r15),%rax
   cd54a:	48 85 c0             	test   %rax,%rax
   cd54d:	74 1c                	je     cd56b <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x46b>
   cd54f:	48 8b 4c 24 08       	mov    0x8(%rsp),%rcx
   cd554:	48 8b 79 20          	mov    0x20(%rcx),%rdi
   cd558:	48 c1 e0 03          	shl    $0x3,%rax
   cd55c:	48 8d 34 40          	lea    (%rax,%rax,2),%rsi
   cd560:	ba 08 00 00 00       	mov    $0x8,%edx
   cd565:	ff 15 65 1d 0a 00    	call   *0xa1d65(%rip)        # 16f2d0 <_DYNAMIC+0x2c0>
   cd56b:	48 81 c4 08 01 00 00 	add    $0x108,%rsp
   cd572:	5b                   	pop    %rbx
   cd573:	41 5c                	pop    %r12
   cd575:	41 5d                	pop    %r13
   cd577:	41 5e                	pop    %r14
   cd579:	41 5f                	pop    %r15
   cd57b:	5d                   	pop    %rbp
   cd57c:	c3                   	ret
   cd57d:	4c 89 6c 24 40       	mov    %r13,0x40(%rsp)
   cd582:	48 8b 4c 24 08       	mov    0x8(%rsp),%rcx
   cd587:	48 8b 01             	mov    (%rcx),%rax
   cd58a:	48 8b 59 08          	mov    0x8(%rcx),%rbx
   cd58e:	48 8b 49 10          	mov    0x10(%rcx),%rcx
   cd592:	48 89 ca             	mov    %rcx,%rdx
   cd595:	48 c1 e2 04          	shl    $0x4,%rdx
   cd599:	48 01 da             	add    %rbx,%rdx
   cd59c:	48 89 5c 24 70       	mov    %rbx,0x70(%rsp)
   cd5a1:	48 89 5c 24 78       	mov    %rbx,0x78(%rsp)
   cd5a6:	48 89 84 24 80 00 00 	mov    %rax,0x80(%rsp)
   cd5ad:	00 
   cd5ae:	48 89 94 24 e8 00 00 	mov    %rdx,0xe8(%rsp)
   cd5b5:	00 
   cd5b6:	48 89 94 24 88 00 00 	mov    %rdx,0x88(%rsp)
   cd5bd:	00 
   cd5be:	48 85 c9             	test   %rcx,%rcx
   cd5c1:	0f 84 eb 01 00 00    	je     cd7b2 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x6b2>
   cd5c7:	4c 8d bc 24 e0 00 00 	lea    0xe0(%rsp),%r15
   cd5ce:	00 
   cd5cf:	eb 0e                	jmp    cd5df <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x4df>
   cd5d1:	48 3b 9c 24 e8 00 00 	cmp    0xe8(%rsp),%rbx
   cd5d8:	00 
   cd5d9:	0f 84 d3 01 00 00    	je     cd7b2 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x6b2>
   cd5df:	48 8d 4b 10          	lea    0x10(%rbx),%rcx
   cd5e3:	48 89 8c 24 00 01 00 	mov    %rcx,0x100(%rsp)
   cd5ea:	00 
   cd5eb:	48 89 4c 24 78       	mov    %rcx,0x78(%rsp)
   cd5f0:	48 8b 0b             	mov    (%rbx),%rcx
   cd5f3:	48 8b 43 08          	mov    0x8(%rbx),%rax
   cd5f7:	48 89 8c 24 90 00 00 	mov    %rcx,0x90(%rsp)
   cd5fe:	00 
   cd5ff:	48 89 8c 24 e0 00 00 	mov    %rcx,0xe0(%rsp)
   cd606:	00 
   cd607:	48 89 44 24 10       	mov    %rax,0x10(%rsp)
   cd60c:	48 89 44 24 68       	mov    %rax,0x68(%rsp)
   cd611:	48 8b ac 24 f8 00 00 	mov    0xf8(%rsp),%rbp
   cd618:	00 
   cd619:	4c 8b 74 24 60       	mov    0x60(%rsp),%r14
   cd61e:	48 83 7c 24 18 00    	cmpq   $0x0,0x18(%rsp)
   cd624:	75 67                	jne    cd68d <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x58d>
   cd626:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
   cd62b:	f0 48 ff 08          	lock decq (%rax)
   cd62f:	75 0b                	jne    cd63c <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x53c>
   cd631:	48 8d 7c 24 68       	lea    0x68(%rsp),%rdi
   cd636:	ff 15 74 1c 0a 00    	call   *0xa1c74(%rip)        # 16f2b0 <_DYNAMIC+0x2a0>
   cd63c:	48 8b 84 24 90 00 00 	mov    0x90(%rsp),%rax
   cd643:	00 
   cd644:	f0 48 ff 08          	lock decq (%rax)
   cd648:	48 8b 9c 24 00 01 00 	mov    0x100(%rsp),%rbx
   cd64f:	00 
   cd650:	0f 85 7b ff ff ff    	jne    cd5d1 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x4d1>
   cd656:	4c 89 ff             	mov    %r15,%rdi
   cd659:	ff 15 51 1c 0a 00    	call   *0xa1c51(%rip)        # 16f2b0 <_DYNAMIC+0x2a0>
   cd65f:	e9 6d ff ff ff       	jmp    cd5d1 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x4d1>
   cd664:	66 66 66 2e 0f 1f 84 	data16 data16 cs nopw 0x0(%rax,%rax,1)
   cd66b:	00 00 00 00 00 
   cd670:	48 89 df             	mov    %rbx,%rdi
   cd673:	4c 89 ee             	mov    %r13,%rsi
   cd676:	e8 a5 27 fc ff       	call   8fe20 <<hyperreal::rational::arithmetic::Rational as core::ops::arith::Add>::add>
   cd67b:	48 89 84 24 d8 00 00 	mov    %rax,0xd8(%rsp)
   cd682:	00 
   cd683:	49 83 c6 10          	add    $0x10,%r14
   cd687:	48 83 c5 f0          	add    $0xfffffffffffffff0,%rbp
   cd68b:	74 99                	je     cd626 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x526>
   cd68d:	49 8d 76 08          	lea    0x8(%r14),%rsi
   cd691:	48 8d 7c 24 68       	lea    0x68(%rsp),%rdi
   cd696:	e8 b5 40 fc ff       	call   91750 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cd69b:	49 89 c4             	mov    %rax,%r12
   cd69e:	48 89 44 24 50       	mov    %rax,0x50(%rsp)
   cd6a3:	4c 89 ff             	mov    %r15,%rdi
   cd6a6:	4c 89 f6             	mov    %r14,%rsi
   cd6a9:	ff 15 a9 1b 0a 00    	call   *0xa1ba9(%rip)        # 16f258 <_DYNAMIC+0x248>
   cd6af:	84 c0                	test   %al,%al
   cd6b1:	74 4d                	je     cd700 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x600>
   cd6b3:	48 8b 9c 24 d8 00 00 	mov    0xd8(%rsp),%rbx
   cd6ba:	00 
   cd6bb:	48 89 5c 24 38       	mov    %rbx,0x38(%rsp)
   cd6c0:	4c 89 a4 24 98 00 00 	mov    %r12,0x98(%rsp)
   cd6c7:	00 
   cd6c8:	48 8d bc 24 98 00 00 	lea    0x98(%rsp),%rdi
   cd6cf:	00 
   cd6d0:	4c 89 fe             	mov    %r15,%rsi
   cd6d3:	e8 78 40 fc ff       	call   91750 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cd6d8:	49 89 c5             	mov    %rax,%r13
   cd6db:	f0 49 ff 0c 24       	lock decq (%r12)
   cd6e0:	75 8e                	jne    cd670 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x570>
   cd6e2:	48 8d bc 24 98 00 00 	lea    0x98(%rsp),%rdi
   cd6e9:	00 
   cd6ea:	ff 15 c0 1b 0a 00    	call   *0xa1bc0(%rip)        # 16f2b0 <_DYNAMIC+0x2a0>
   cd6f0:	e9 7b ff ff ff       	jmp    cd670 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x570>
   cd6f5:	66 66 2e 0f 1f 84 00 	data16 cs nopw 0x0(%rax,%rax,1)
   cd6fc:	00 00 00 00 
   cd700:	4c 89 ff             	mov    %r15,%rdi
   cd703:	4c 89 f6             	mov    %r14,%rsi
   cd706:	e8 45 40 fc ff       	call   91750 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cd70b:	48 89 c7             	mov    %rax,%rdi
   cd70e:	e8 0d 5c fa ff       	call   73320 <<hyperreal::rational::arithmetic::Rational>::extract_square_reduced_retained>
   cd713:	48 89 d3             	mov    %rdx,%rbx
   cd716:	48 89 94 24 98 00 00 	mov    %rdx,0x98(%rsp)
   cd71d:	00 
   cd71e:	4c 89 e7             	mov    %r12,%rdi
   cd721:	48 89 c6             	mov    %rax,%rsi
   cd724:	e8 a7 4b fc ff       	call   922d0 <<hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cd729:	48 8d bc 24 a8 00 00 	lea    0xa8(%rsp),%rdi
   cd730:	00 
   cd731:	48 89 de             	mov    %rbx,%rsi
   cd734:	48 89 c2             	mov    %rax,%rdx
   cd737:	e8 14 f1 ff ff       	call   cc850 <<hyperreal::computable::node::RadicalCoefficient>::insert_radical>
   cd73c:	84 c0                	test   %al,%al
   cd73e:	0f 85 3f ff ff ff    	jne    cd683 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x583>
   cd744:	48 8b 44 24 40       	mov    0x40(%rsp),%rax
   cd749:	48 c7 00 ff ff ff ff 	movq   $0xffffffffffffffff,(%rax)
   cd750:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
   cd755:	f0 48 ff 08          	lock decq (%rax)
   cd759:	75 0b                	jne    cd766 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x666>
   cd75b:	48 8d 7c 24 68       	lea    0x68(%rsp),%rdi
   cd760:	ff 15 4a 1b 0a 00    	call   *0xa1b4a(%rip)        # 16f2b0 <_DYNAMIC+0x2a0>
   cd766:	48 8b 84 24 90 00 00 	mov    0x90(%rsp),%rax
   cd76d:	00 
   cd76e:	f0 48 ff 08          	lock decq (%rax)
   cd772:	75 0e                	jne    cd782 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x682>
   cd774:	48 8d bc 24 e0 00 00 	lea    0xe0(%rsp),%rdi
   cd77b:	00 
   cd77c:	ff 15 2e 1b 0a 00    	call   *0xa1b2e(%rip)        # 16f2b0 <_DYNAMIC+0x2a0>
   cd782:	b3 01                	mov    $0x1,%bl
   cd784:	48 8d 7c 24 70       	lea    0x70(%rsp),%rdi
   cd789:	e8 32 01 04 00       	call   10d8c0 <<alloc::vec::into_iter::IntoIter<(hyperreal::rational::arithmetic::Rational, hyperreal::rational::arithmetic::Rational)> as core::ops::drop::Drop>::drop>
   cd78e:	45 31 f6             	xor    %r14d,%r14d
   cd791:	e9 2e fd ff ff       	jmp    cd4c4 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x3c4>
   cd796:	b0 01                	mov    $0x1,%al
   cd798:	89 44 24 2c          	mov    %eax,0x2c(%rsp)
   cd79c:	48 c7 44 24 30 00 00 	movq   $0x0,0x30(%rsp)
   cd7a3:	00 00 
   cd7a5:	48 8b 7c 24 08       	mov    0x8(%rsp),%rdi
   cd7aa:	4c 89 e2             	mov    %r12,%rdx
   cd7ad:	e9 c8 fb ff ff       	jmp    cd37a <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x27a>
   cd7b2:	b3 01                	mov    $0x1,%bl
   cd7b4:	48 8d 7c 24 70       	lea    0x70(%rsp),%rdi
   cd7b9:	e8 02 01 04 00       	call   10d8c0 <<alloc::vec::into_iter::IntoIter<(hyperreal::rational::arithmetic::Rational, hyperreal::rational::arithmetic::Rational)> as core::ops::drop::Drop>::drop>
   cd7be:	48 8b 4c 24 08       	mov    0x8(%rsp),%rcx
   cd7c3:	48 8b 41 18          	mov    0x18(%rcx),%rax
   cd7c7:	4c 8b 69 20          	mov    0x20(%rcx),%r13
   cd7cb:	48 8b 49 28          	mov    0x28(%rcx),%rcx
   cd7cf:	48 8d 14 cd 00 00 00 	lea    0x0(,%rcx,8),%rdx
   cd7d6:	00 
   cd7d7:	4c 8d 3c 52          	lea    (%rdx,%rdx,2),%r15
   cd7db:	4b 8d 54 3d 00       	lea    0x0(%r13,%r15,1),%rdx
   cd7e0:	4c 89 6c 24 70       	mov    %r13,0x70(%rsp)
   cd7e5:	4c 89 6c 24 78       	mov    %r13,0x78(%rsp)
   cd7ea:	48 89 84 24 80 00 00 	mov    %rax,0x80(%rsp)
   cd7f1:	00 
   cd7f2:	48 89 94 24 88 00 00 	mov    %rdx,0x88(%rsp)
   cd7f9:	00 
   cd7fa:	48 85 c9             	test   %rcx,%rcx
   cd7fd:	74 7f                	je     cd87e <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x77e>
   cd7ff:	49 83 c5 18          	add    $0x18,%r13
   cd803:	66 66 66 66 2e 0f 1f 	data16 data16 data16 cs nopw 0x0(%rax,%rax,1)
   cd80a:	84 00 00 00 00 00 
   cd810:	4c 89 6c 24 78       	mov    %r13,0x78(%rsp)
   cd815:	49 8b 5d e8          	mov    -0x18(%r13),%rbx
   cd819:	48 85 db             	test   %rbx,%rbx
   cd81c:	74 60                	je     cd87e <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x77e>
   cd81e:	4d 8b 65 f0          	mov    -0x10(%r13),%r12
   cd822:	4d 8b 75 f8          	mov    -0x8(%r13),%r14
   cd826:	48 89 9c 24 98 00 00 	mov    %rbx,0x98(%rsp)
   cd82d:	00 
   cd82e:	4c 89 a4 24 a0 00 00 	mov    %r12,0xa0(%rsp)
   cd835:	00 
   cd836:	4c 89 74 24 38       	mov    %r14,0x38(%rsp)
   cd83b:	48 8d 7c 24 38       	lea    0x38(%rsp),%rdi
   cd840:	48 8b 74 24 20       	mov    0x20(%rsp),%rsi
   cd845:	e8 06 3f fc ff       	call   91750 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cd84a:	48 89 c5             	mov    %rax,%rbp
   cd84d:	f0 49 ff 0e          	lock decq (%r14)
   cd851:	75 0b                	jne    cd85e <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x75e>
   cd853:	48 8d 7c 24 38       	lea    0x38(%rsp),%rdi
   cd858:	ff 15 52 1a 0a 00    	call   *0xa1a52(%rip)        # 16f2b0 <_DYNAMIC+0x2a0>
   cd85e:	48 8d bc 24 a8 00 00 	lea    0xa8(%rsp),%rdi
   cd865:	00 
   cd866:	48 89 de             	mov    %rbx,%rsi
   cd869:	4c 89 e2             	mov    %r12,%rdx
   cd86c:	48 89 e9             	mov    %rbp,%rcx
   cd86f:	e8 8c ec ff ff       	call   cc500 <<hyperreal::computable::node::RadicalCoefficient>::insert_atom>
   cd874:	49 83 c5 18          	add    $0x18,%r13
   cd878:	49 83 c7 e8          	add    $0xffffffffffffffe8,%r15
   cd87c:	75 92                	jne    cd810 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x710>
   cd87e:	31 db                	xor    %ebx,%ebx
   cd880:	48 8d 7c 24 70       	lea    0x70(%rsp),%rdi
   cd885:	e8 66 ff 03 00       	call   10d7f0 <<alloc::vec::into_iter::IntoIter<(hyperreal::computable::node::Computable, hyperreal::rational::arithmetic::Rational)> as core::ops::drop::Drop>::drop>
   cd88a:	80 7c 24 2c 00       	cmpb   $0x0,0x2c(%rsp)
   cd88f:	4c 8b 6c 24 40       	mov    0x40(%rsp),%r13
   cd894:	75 72                	jne    cd908 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x808>
   cd896:	48 8b 84 24 f0 00 00 	mov    0xf0(%rsp),%rax
   cd89d:	00 
   cd89e:	48 8b 58 20          	mov    0x20(%rax),%rbx
   cd8a2:	48 01 5c 24 30       	add    %rbx,0x30(%rsp)
   cd8a7:	4c 8d b4 24 a8 00 00 	lea    0xa8(%rsp),%r14
   cd8ae:	00 
   cd8af:	90                   	nop
   cd8b0:	4c 8b 3b             	mov    (%rbx),%r15
   cd8b3:	4c 8b 63 08          	mov    0x8(%rbx),%r12
   cd8b7:	f0 49 ff 07          	lock incq (%r15)
   cd8bb:	0f 8e ff 03 00 00    	jle    cdcc0 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xbc0>
   cd8c1:	4d 85 e4             	test   %r12,%r12
   cd8c4:	74 0b                	je     cd8d1 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x7d1>
   cd8c6:	f0 49 ff 04 24       	lock incq (%r12)
   cd8cb:	0f 8e ef 03 00 00    	jle    cdcc0 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xbc0>
   cd8d1:	48 83 c3 10          	add    $0x10,%rbx
   cd8d5:	4c 89 7c 24 70       	mov    %r15,0x70(%rsp)
   cd8da:	4c 89 64 24 78       	mov    %r12,0x78(%rsp)
   cd8df:	48 89 df             	mov    %rbx,%rdi
   cd8e2:	48 8b 74 24 48       	mov    0x48(%rsp),%rsi
   cd8e7:	e8 64 3e fc ff       	call   91750 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cd8ec:	4c 89 f7             	mov    %r14,%rdi
   cd8ef:	4c 89 fe             	mov    %r15,%rsi
   cd8f2:	4c 89 e2             	mov    %r12,%rdx
   cd8f5:	48 89 c1             	mov    %rax,%rcx
   cd8f8:	e8 03 ec ff ff       	call   cc500 <<hyperreal::computable::node::RadicalCoefficient>::insert_atom>
   cd8fd:	48 83 c3 08          	add    $0x8,%rbx
   cd901:	48 3b 5c 24 30       	cmp    0x30(%rsp),%rbx
   cd906:	75 a8                	jne    cd8b0 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x7b0>
   cd908:	48 8b 84 24 d8 00 00 	mov    0xd8(%rsp),%rax
   cd90f:	00 
   cd910:	49 89 45 30          	mov    %rax,0x30(%r13)
   cd914:	0f 10 84 24 a8 00 00 	movups 0xa8(%rsp),%xmm0
   cd91b:	00 
   cd91c:	0f 10 8c 24 b8 00 00 	movups 0xb8(%rsp),%xmm1
   cd923:	00 
   cd924:	0f 10 94 24 c8 00 00 	movups 0xc8(%rsp),%xmm2
   cd92b:	00 
   cd92c:	41 0f 11 55 20       	movups %xmm2,0x20(%r13)
   cd931:	41 0f 11 4d 10       	movups %xmm1,0x10(%r13)
   cd936:	41 0f 11 45 00       	movups %xmm0,0x0(%r13)
   cd93b:	48 8b 7c 24 48       	mov    0x48(%rsp),%rdi
   cd940:	48 8b 07             	mov    (%rdi),%rax
   cd943:	f0 48 ff 08          	lock decq (%rax)
   cd947:	0f 85 1e fc ff ff    	jne    cd56b <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x46b>
   cd94d:	ff 15 5d 19 0a 00    	call   *0xa195d(%rip)        # 16f2b0 <_DYNAMIC+0x2a0>
   cd953:	e9 13 fc ff ff       	jmp    cd56b <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x46b>
   cd958:	48 89 de             	mov    %rbx,%rsi
   cd95b:	eb 18                	jmp    cd975 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x875>
   cd95d:	0f 1f 00             	nopl   (%rax)
   cd960:	49 89 f7             	mov    %rsi,%r15
   cd963:	4c 89 fe             	mov    %r15,%rsi
   cd966:	4d 8b 3f             	mov    (%r15),%r15
   cd969:	41 8b 47 60          	mov    0x60(%r15),%eax
   cd96d:	84 c0                	test   %al,%al
   cd96f:	0f 89 a2 f8 ff ff    	jns    cd217 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x117>
   cd975:	41 8b 47 60          	mov    0x60(%r15),%eax
   cd979:	84 c0                	test   %al,%al
   cd97b:	79 e3                	jns    cd960 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x860>
   cd97d:	41 8b 47 50          	mov    0x50(%r15),%eax
   cd981:	49 83 c7 40          	add    $0x40,%r15
   cd985:	85 c0                	test   %eax,%eax
   cd987:	74 da                	je     cd963 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x863>
   cd989:	4c 89 ff             	mov    %r15,%rdi
   cd98c:	e8 18 04 04 00       	call   10dda9 <<std::sync::once_lock::OnceLock<hyperreal::rational::arithmetic::CachedRationalProduct>>::initialize::<<std::sync::once_lock::OnceLock<hyperreal::rational::arithmetic::CachedRationalProduct>>::get_or_init<<hyperreal::rational::arithmetic::Rational>::canonicalized_ref::{closure#0}>::{closure#0}, !>>
   cd991:	eb d0                	jmp    cd963 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x863>
   cd993:	48 89 44 24 20       	mov    %rax,0x20(%rsp)
   cd998:	80 7e 64 01          	cmpb   $0x1,0x64(%rsi)
   cd99c:	75 08                	jne    cd9a6 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x8a6>
   cd99e:	b0 01                	mov    $0x1,%al
   cd9a0:	89 44 24 10          	mov    %eax,0x10(%rsp)
   cd9a4:	eb 26                	jmp    cd9cc <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x8cc>
   cd9a6:	c7 44 24 10 00 00 00 	movl   $0x0,0x10(%rsp)
   cd9ad:	00 
   cd9ae:	31 db                	xor    %ebx,%ebx
   cd9b0:	48 8b 7c 24 20       	mov    0x20(%rsp),%rdi
   cd9b5:	e8 66 24 fc ff       	call   8fe20 <<hyperreal::rational::arithmetic::Rational as core::ops::arith::Add>::add>
   cd9ba:	48 89 44 24 20       	mov    %rax,0x20(%rsp)
   cd9bf:	48 89 44 24 50       	mov    %rax,0x50(%rsp)
   cd9c4:	c7 44 24 10 00 00 00 	movl   $0x0,0x10(%rsp)
   cd9cb:	00 
   cd9cc:	b3 01                	mov    $0x1,%bl
   cd9ce:	48 8b bc 24 90 00 00 	mov    0x90(%rsp),%rdi
   cd9d5:	00 
   cd9d6:	4c 89 f6             	mov    %r14,%rsi
   cd9d9:	e8 72 3d fc ff       	call   91750 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cd9de:	49 89 c6             	mov    %rax,%r14
   cd9e1:	48 8b 44 24 48       	mov    0x48(%rsp),%rax
   cd9e6:	48 89 44 24 70       	mov    %rax,0x70(%rsp)
   cd9eb:	4c 89 b4 24 a8 00 00 	mov    %r14,0xa8(%rsp)
   cd9f2:	00 
   cd9f3:	48 8d bc 24 a8 00 00 	lea    0xa8(%rsp),%rdi
   cd9fa:	00 
   cd9fb:	4c 89 fe             	mov    %r15,%rsi
   cd9fe:	e8 4d 3d fc ff       	call   91750 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cda03:	48 89 c3             	mov    %rax,%rbx
   cda06:	f0 49 ff 0e          	lock decq (%r14)
   cda0a:	4c 8b 74 24 60       	mov    0x60(%rsp),%r14
   cda0f:	75 0e                	jne    cda1f <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x91f>
   cda11:	48 8d bc 24 a8 00 00 	lea    0xa8(%rsp),%rdi
   cda18:	00 
   cda19:	ff 15 91 18 0a 00    	call   *0xa1891(%rip)        # 16f2b0 <_DYNAMIC+0x2a0>
   cda1f:	48 8b 7c 24 48       	mov    0x48(%rsp),%rdi
   cda24:	48 89 de             	mov    %rbx,%rsi
   cda27:	e8 f4 23 fc ff       	call   8fe20 <<hyperreal::rational::arithmetic::Rational as core::ops::arith::Add>::add>
   cda2c:	48 89 c3             	mov    %rax,%rbx
   cda2f:	48 89 84 24 98 00 00 	mov    %rax,0x98(%rsp)
   cda36:	00 
   cda37:	48 8b 44 24 20       	mov    0x20(%rsp),%rax
   cda3c:	80 78 64 01          	cmpb   $0x1,0x64(%rax)
   cda40:	0f 85 87 00 00 00    	jne    cdacd <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x9cd>
   cda46:	48 8b 44 24 40       	mov    0x40(%rsp),%rax
   cda4b:	48 c7 00 00 00 00 00 	movq   $0x0,(%rax)
   cda52:	48 c7 40 08 08 00 00 	movq   $0x8,0x8(%rax)
   cda59:	00 
   cda5a:	0f 57 c0             	xorps  %xmm0,%xmm0
   cda5d:	0f 11 40 10          	movups %xmm0,0x10(%rax)
   cda61:	48 c7 40 20 08 00 00 	movq   $0x8,0x20(%rax)
   cda68:	00 
   cda69:	48 c7 40 28 00 00 00 	movq   $0x0,0x28(%rax)
   cda70:	00 
   cda71:	48 89 58 30          	mov    %rbx,0x30(%rax)
   cda75:	80 7c 24 10 00       	cmpb   $0x0,0x10(%rsp)
   cda7a:	4c 8b 64 24 20       	mov    0x20(%rsp),%r12
   cda7f:	48 8b 44 24 18       	mov    0x18(%rsp),%rax
   cda84:	74 1a                	je     cdaa0 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x9a0>
   cda86:	f0 48 ff 08          	lock decq (%rax)
   cda8a:	75 14                	jne    cdaa0 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x9a0>
   cda8c:	b0 01                	mov    $0x1,%al
   cda8e:	89 44 24 10          	mov    %eax,0x10(%rsp)
   cda92:	45 31 ff             	xor    %r15d,%r15d
   cda95:	48 8d 7c 24 38       	lea    0x38(%rsp),%rdi
   cda9a:	ff 15 10 18 0a 00    	call   *0xa1810(%rip)        # 16f2b0 <_DYNAMIC+0x2a0>
   cdaa0:	f0 49 ff 0c 24       	lock decq (%r12)
   cdaa5:	75 0b                	jne    cdab2 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x9b2>
   cdaa7:	48 8d 7c 24 50       	lea    0x50(%rsp),%rdi
   cdaac:	ff 15 fe 17 0a 00    	call   *0xa17fe(%rip)        # 16f2b0 <_DYNAMIC+0x2a0>
   cdab2:	41 b6 01             	mov    $0x1,%r14b
   cdab5:	48 8b 5c 24 08       	mov    0x8(%rsp),%rbx
   cdaba:	48 8b 43 30          	mov    0x30(%rbx),%rax
   cdabe:	f0 48 ff 08          	lock decq (%rax)
   cdac2:	0f 84 18 fa ff ff    	je     cd4e0 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x3e0>
   cdac8:	e9 1d fa ff ff       	jmp    cd4ea <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x3ea>
   cdacd:	41 80 fc 01          	cmp    $0x1,%r12b
   cdad1:	75 3c                	jne    cdb0f <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xa0f>
   cdad3:	49 8b 06             	mov    (%r14),%rax
   cdad6:	f0 48 ff 00          	lock incq (%rax)
   cdada:	0f 8e e0 01 00 00    	jle    cdcc0 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xbc0>
   cdae0:	4d 8b 26             	mov    (%r14),%r12
   cdae3:	4c 89 a4 24 a8 00 00 	mov    %r12,0xa8(%rsp)
   cdaea:	00 
   cdaeb:	48 83 7d 00 00       	cmpq   $0x0,0x0(%rbp)
   cdaf0:	0f 84 b9 01 00 00    	je     cdcaf <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xbaf>
   cdaf6:	4d 8b 75 00          	mov    0x0(%r13),%r14
   cdafa:	49 8b 06             	mov    (%r14),%rax
   cdafd:	f0 48 ff 08          	lock decq (%rax)
   cdb01:	75 09                	jne    cdb0c <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xa0c>
   cdb03:	4c 89 f7             	mov    %r14,%rdi
   cdb06:	ff 15 a4 17 0a 00    	call   *0xa17a4(%rip)        # 16f2b0 <_DYNAMIC+0x2a0>
   cdb0c:	4d 89 26             	mov    %r12,(%r14)
   cdb0f:	4c 8b 74 24 30       	mov    0x30(%rsp),%r14
   cdb14:	49 8b 06             	mov    (%r14),%rax
   cdb17:	f0 48 ff 08          	lock decq (%rax)
   cdb1b:	4c 8b 64 24 20       	mov    0x20(%rsp),%r12
   cdb20:	75 09                	jne    cdb2b <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xa2b>
   cdb22:	4c 89 f7             	mov    %r14,%rdi
   cdb25:	ff 15 85 17 0a 00    	call   *0xa1785(%rip)        # 16f2b0 <_DYNAMIC+0x2a0>
   cdb2b:	49 89 1e             	mov    %rbx,(%r14)
   cdb2e:	4c 89 a4 24 a8 00 00 	mov    %r12,0xa8(%rsp)
   cdb35:	00 
   cdb36:	48 83 7d 00 00       	cmpq   $0x0,0x0(%rbp)
   cdb3b:	0f 84 5b 01 00 00    	je     cdc9c <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xb9c>
   cdb41:	49 8b 5d 00          	mov    0x0(%r13),%rbx
   cdb45:	48 8b 43 08          	mov    0x8(%rbx),%rax
   cdb49:	48 83 c3 08          	add    $0x8,%rbx
   cdb4d:	f0 48 ff 08          	lock decq (%rax)
   cdb51:	4c 8b 74 24 40       	mov    0x40(%rsp),%r14
   cdb56:	75 09                	jne    cdb61 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xa61>
   cdb58:	48 89 df             	mov    %rbx,%rdi
   cdb5b:	ff 15 4f 17 0a 00    	call   *0xa174f(%rip)        # 16f2b0 <_DYNAMIC+0x2a0>
   cdb61:	4c 89 23             	mov    %r12,(%rbx)
   cdb64:	48 8b 5c 24 08       	mov    0x8(%rsp),%rbx
   cdb69:	48 8d 7b 18          	lea    0x18(%rbx),%rdi
   cdb6d:	e8 de 1d fe ff       	call   af950 <core::ptr::drop_glue::<alloc::vec::Vec<(hyperreal::computable::node::Computable, hyperreal::rational::arithmetic::Rational)>>>
   cdb72:	48 c7 43 18 00 00 00 	movq   $0x0,0x18(%rbx)
   cdb79:	00 
   cdb7a:	48 c7 43 20 08 00 00 	movq   $0x8,0x20(%rbx)
   cdb81:	00 
   cdb82:	48 c7 43 28 00 00 00 	movq   $0x0,0x28(%rbx)
   cdb89:	00 
   cdb8a:	48 8b 43 30          	mov    0x30(%rbx),%rax
   cdb8e:	49 89 46 30          	mov    %rax,0x30(%r14)
   cdb92:	0f 10 03             	movups (%rbx),%xmm0
   cdb95:	41 0f 11 06          	movups %xmm0,(%r14)
   cdb99:	48 8b 43 20          	mov    0x20(%rbx),%rax
   cdb9d:	49 89 46 20          	mov    %rax,0x20(%r14)
   cdba1:	48 8b 43 28          	mov    0x28(%rbx),%rax
   cdba5:	49 89 46 28          	mov    %rax,0x28(%r14)
   cdba9:	48 8b 43 10          	mov    0x10(%rbx),%rax
   cdbad:	49 89 46 10          	mov    %rax,0x10(%r14)
   cdbb1:	48 8b 43 18          	mov    0x18(%rbx),%rax
   cdbb5:	49 89 46 18          	mov    %rax,0x18(%r14)
   cdbb9:	80 7c 24 10 00       	cmpb   $0x0,0x10(%rsp)
   cdbbe:	48 8b 44 24 18       	mov    0x18(%rsp),%rax
   cdbc3:	0f 84 a2 f9 ff ff    	je     cd56b <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x46b>
   cdbc9:	f0 48 ff 08          	lock decq (%rax)
   cdbcd:	0f 85 98 f9 ff ff    	jne    cd56b <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x46b>
   cdbd3:	c7 44 24 10 00 00 00 	movl   $0x0,0x10(%rsp)
   cdbda:	00 
   cdbdb:	48 8d 7c 24 38       	lea    0x38(%rsp),%rdi
   cdbe0:	45 31 ff             	xor    %r15d,%r15d
   cdbe3:	ff 15 c7 16 0a 00    	call   *0xa16c7(%rip)        # 16f2b0 <_DYNAMIC+0x2a0>
   cdbe9:	e9 7d f9 ff ff       	jmp    cd56b <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x46b>
   cdbee:	4c 89 74 24 70       	mov    %r14,0x70(%rsp)
   cdbf3:	48 8d 44 24 70       	lea    0x70(%rsp),%rax
   cdbf8:	48 89 84 24 a8 00 00 	mov    %rax,0xa8(%rsp)
   cdbff:	00 
   cdc00:	4c 89 f0             	mov    %r14,%rax
   cdc03:	41 b6 01             	mov    $0x1,%r14b
   cdc06:	48 8d 78 18          	lea    0x18(%rax),%rdi
   cdc0a:	48 8d 0d 8f d3 09 00 	lea    0x9d38f(%rip),%rcx        # 16afa0 <anon.afeede234644f26776946e76b404b106.3.llvm.7695097067950034038+0x28>
   cdc11:	4c 8d 05 48 d3 09 00 	lea    0x9d348(%rip),%r8        # 16af60 <anon.afeede234644f26776946e76b404b106.2.llvm.7695097067950034038>
   cdc18:	48 8d 94 24 a8 00 00 	lea    0xa8(%rsp),%rdx
   cdc1f:	00 
   cdc20:	be 01 00 00 00       	mov    $0x1,%esi
   cdc25:	ff 15 25 16 0a 00    	call   *0xa1625(%rip)        # 16f250 <_DYNAMIC+0x240>
   cdc2b:	4c 8b 35 66 16 0a 00 	mov    0xa1666(%rip),%r14        # 16f298 <_DYNAMIC+0x288>
   cdc32:	49 8b 57 20          	mov    0x20(%r15),%rdx
   cdc36:	49 3b 56 10          	cmp    0x10(%r14),%rdx
   cdc3a:	0f 84 fb f5 ff ff    	je     cd23b <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x13b>
   cdc40:	e9 3c f6 ff ff       	jmp    cd281 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x181>
   cdc45:	4c 89 74 24 70       	mov    %r14,0x70(%rsp)
   cdc4a:	48 8d 44 24 70       	lea    0x70(%rsp),%rax
   cdc4f:	48 89 84 24 a8 00 00 	mov    %rax,0xa8(%rsp)
   cdc56:	00 
   cdc57:	4c 89 f0             	mov    %r14,%rax
   cdc5a:	41 b6 01             	mov    $0x1,%r14b
   cdc5d:	48 8d 78 18          	lea    0x18(%rax),%rdi
   cdc61:	48 8d 0d 38 d3 09 00 	lea    0x9d338(%rip),%rcx        # 16afa0 <anon.afeede234644f26776946e76b404b106.3.llvm.7695097067950034038+0x28>
   cdc68:	4c 8d 05 f1 d2 09 00 	lea    0x9d2f1(%rip),%r8        # 16af60 <anon.afeede234644f26776946e76b404b106.2.llvm.7695097067950034038>
   cdc6f:	48 8d 94 24 a8 00 00 	lea    0xa8(%rsp),%rdx
   cdc76:	00 
   cdc77:	be 01 00 00 00       	mov    $0x1,%esi
   cdc7c:	ff 15 ce 15 0a 00    	call   *0xa15ce(%rip)        # 16f250 <_DYNAMIC+0x240>
   cdc82:	4c 8b 35 0f 16 0a 00 	mov    0xa160f(%rip),%r14        # 16f298 <_DYNAMIC+0x288>
   cdc89:	49 8b 57 38          	mov    0x38(%r15),%rdx
   cdc8d:	49 3b 56 10          	cmp    0x10(%r14),%rdx
   cdc91:	0f 84 d0 f5 ff ff    	je     cd267 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x167>
   cdc97:	e9 e5 f5 ff ff       	jmp    cd281 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x181>
   cdc9c:	48 8d 15 e5 d9 09 00 	lea    0x9d9e5(%rip),%rdx        # 16b688 <anon.afeede234644f26776946e76b404b106.261.llvm.7695097067950034038+0x140>
   cdca3:	31 ff                	xor    %edi,%edi
   cdca5:	31 f6                	xor    %esi,%esi
   cdca7:	ff 15 e3 1a 0a 00    	call   *0xa1ae3(%rip)        # 16f790 <_DYNAMIC+0x780>
   cdcad:	eb 11                	jmp    cdcc0 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xbc0>
   cdcaf:	48 8d 15 ba d9 09 00 	lea    0x9d9ba(%rip),%rdx        # 16b670 <anon.afeede234644f26776946e76b404b106.261.llvm.7695097067950034038+0x128>
   cdcb6:	31 ff                	xor    %edi,%edi
   cdcb8:	31 f6                	xor    %esi,%esi
   cdcba:	ff 15 d0 1a 0a 00    	call   *0xa1ad0(%rip)        # 16f790 <_DYNAMIC+0x780>
   cdcc0:	0f 0b                	ud2
   cdcc2:	eb 55                	jmp    cdd19 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xc19>
   cdcc4:	49 89 c7             	mov    %rax,%r15
   cdcc7:	4d 89 26             	mov    %r12,(%r14)
   cdcca:	eb 2e                	jmp    cdcfa <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xbfa>
   cdccc:	49 89 c7             	mov    %rax,%r15
   cdccf:	4c 89 23             	mov    %r12,(%rbx)
   cdcd2:	e9 82 00 00 00       	jmp    cdd59 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xc59>
   cdcd7:	49 89 c7             	mov    %rax,%r15
   cdcda:	49 89 1e             	mov    %rbx,(%r14)
   cdcdd:	e9 c2 00 00 00       	jmp    cdda4 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xca4>
   cdce2:	49 89 c7             	mov    %rax,%r15
   cdce5:	f0 49 ff 0c 24       	lock decq (%r12)
   cdcea:	75 0e                	jne    cdcfa <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xbfa>
   cdcec:	48 8d bc 24 a8 00 00 	lea    0xa8(%rsp),%rdi
   cdcf3:	00 
   cdcf4:	ff 15 b6 15 0a 00    	call   *0xa15b6(%rip)        # 16f2b0 <_DYNAMIC+0x2a0>
   cdcfa:	f0 48 ff 0b          	lock decq (%rbx)
   cdcfe:	b3 01                	mov    $0x1,%bl
   cdd00:	0f 85 d4 00 00 00    	jne    cddda <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xcda>
   cdd06:	48 8d bc 24 98 00 00 	lea    0x98(%rsp),%rdi
   cdd0d:	00 
   cdd0e:	ff 15 9c 15 0a 00    	call   *0xa159c(%rip)        # 16f2b0 <_DYNAMIC+0x2a0>
   cdd14:	e9 c1 00 00 00       	jmp    cddda <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xcda>
   cdd19:	49 89 c7             	mov    %rax,%r15
   cdd1c:	b3 01                	mov    $0x1,%bl
   cdd1e:	41 b6 01             	mov    $0x1,%r14b
   cdd21:	e9 7f 03 00 00       	jmp    ce0a5 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xfa5>
   cdd26:	49 89 c7             	mov    %rax,%r15
   cdd29:	48 8b 44 24 18       	mov    0x18(%rsp),%rax
   cdd2e:	48 89 44 24 50       	mov    %rax,0x50(%rsp)
   cdd33:	41 b5 01             	mov    $0x1,%r13b
   cdd36:	40 b5 01             	mov    $0x1,%bpl
   cdd39:	e9 f8 00 00 00       	jmp    cde36 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xd36>
   cdd3e:	49 89 c7             	mov    %rax,%r15
   cdd41:	48 c7 43 18 00 00 00 	movq   $0x0,0x18(%rbx)
   cdd48:	00 
   cdd49:	48 c7 43 20 08 00 00 	movq   $0x8,0x20(%rbx)
   cdd50:	00 
   cdd51:	48 c7 43 28 00 00 00 	movq   $0x0,0x28(%rbx)
   cdd58:	00 
   cdd59:	31 db                	xor    %ebx,%ebx
   cdd5b:	45 31 ed             	xor    %r13d,%r13d
   cdd5e:	e9 a5 00 00 00       	jmp    cde08 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xd08>
   cdd63:	49 89 c7             	mov    %rax,%r15
   cdd66:	45 31 ed             	xor    %r13d,%r13d
   cdd69:	48 8b 44 24 20       	mov    0x20(%rsp),%rax
   cdd6e:	f0 48 ff 08          	lock decq (%rax)
   cdd72:	bb 00 00 00 00       	mov    $0x0,%ebx
   cdd77:	0f 85 8b 00 00 00    	jne    cde08 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xd08>
   cdd7d:	31 db                	xor    %ebx,%ebx
   cdd7f:	48 8d bc 24 a8 00 00 	lea    0xa8(%rsp),%rdi
   cdd86:	00 
   cdd87:	ff 15 23 15 0a 00    	call   *0xa1523(%rip)        # 16f2b0 <_DYNAMIC+0x2a0>
   cdd8d:	45 31 ed             	xor    %r13d,%r13d
   cdd90:	eb 76                	jmp    cde08 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xd08>
   cdd92:	49 89 c7             	mov    %rax,%r15
   cdd95:	eb 2b                	jmp    cddc2 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xcc2>
   cdd97:	e9 de 00 00 00       	jmp    cde7a <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xd7a>
   cdd9c:	e9 e4 00 00 00       	jmp    cde85 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xd85>
   cdda1:	49 89 c7             	mov    %rax,%r15
   cdda4:	b3 01                	mov    $0x1,%bl
   cdda6:	45 31 ed             	xor    %r13d,%r13d
   cdda9:	eb 5d                	jmp    cde08 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xd08>
   cddab:	49 89 c7             	mov    %rax,%r15
   cddae:	f0 49 ff 0e          	lock decq (%r14)
   cddb2:	75 0e                	jne    cddc2 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xcc2>
   cddb4:	48 8d bc 24 a8 00 00 	lea    0xa8(%rsp),%rdi
   cddbb:	00 
   cddbc:	ff 15 ee 14 0a 00    	call   *0xa14ee(%rip)        # 16f2b0 <_DYNAMIC+0x2a0>
   cddc2:	48 8b 44 24 48       	mov    0x48(%rsp),%rax
   cddc7:	f0 48 ff 08          	lock decq (%rax)
   cddcb:	b3 01                	mov    $0x1,%bl
   cddcd:	75 0b                	jne    cddda <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xcda>
   cddcf:	48 8d 7c 24 70       	lea    0x70(%rsp),%rdi
   cddd4:	ff 15 d6 14 0a 00    	call   *0xa14d6(%rip)        # 16f2b0 <_DYNAMIC+0x2a0>
   cddda:	45 31 ed             	xor    %r13d,%r13d
   cdddd:	eb 29                	jmp    cde08 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xd08>
   cdddf:	ff 15 d3 14 0a 00    	call   *0xa14d3(%rip)        # 16f2b8 <_DYNAMIC+0x2a8>
   cdde5:	49 89 c7             	mov    %rax,%r15
   cdde8:	40 b5 01             	mov    $0x1,%bpl
   cddeb:	eb 64                	jmp    cde51 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xd51>
   cdded:	45 89 fd             	mov    %r15d,%r13d
   cddf0:	4c 89 64 24 20       	mov    %r12,0x20(%rsp)
   cddf5:	49 89 c7             	mov    %rax,%r15
   cddf8:	8b 44 24 10          	mov    0x10(%rsp),%eax
   cddfc:	89 c3                	mov    %eax,%ebx
   cddfe:	89 c5                	mov    %eax,%ebp
   cde00:	eb 26                	jmp    cde28 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xd28>
   cde02:	49 89 c7             	mov    %rax,%r15
   cde05:	41 b5 01             	mov    $0x1,%r13b
   cde08:	40 b5 01             	mov    $0x1,%bpl
   cde0b:	80 7c 24 10 00       	cmpb   $0x0,0x10(%rsp)
   cde10:	74 16                	je     cde28 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xd28>
   cde12:	48 8b 44 24 18       	mov    0x18(%rsp),%rax
   cde17:	f0 48 ff 08          	lock decq (%rax)
   cde1b:	75 0b                	jne    cde28 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xd28>
   cde1d:	48 8d 7c 24 38       	lea    0x38(%rsp),%rdi
   cde22:	ff 15 88 14 0a 00    	call   *0xa1488(%rip)        # 16f2b0 <_DYNAMIC+0x2a0>
   cde28:	84 db                	test   %bl,%bl
   cde2a:	74 20                	je     cde4c <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xd4c>
   cde2c:	48 8b 44 24 20       	mov    0x20(%rsp),%rax
   cde31:	48 89 44 24 18       	mov    %rax,0x18(%rsp)
   cde36:	48 8b 44 24 18       	mov    0x18(%rsp),%rax
   cde3b:	f0 48 ff 08          	lock decq (%rax)
   cde3f:	75 0b                	jne    cde4c <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xd4c>
   cde41:	48 8d 7c 24 50       	lea    0x50(%rsp),%rdi
   cde46:	ff 15 64 14 0a 00    	call   *0xa1464(%rip)        # 16f2b0 <_DYNAMIC+0x2a0>
   cde4c:	45 84 ed             	test   %r13b,%r13b
   cde4f:	74 16                	je     cde67 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xd67>
   cde51:	48 8b 44 24 48       	mov    0x48(%rsp),%rax
   cde56:	f0 48 ff 08          	lock decq (%rax)
   cde5a:	75 0b                	jne    cde67 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xd67>
   cde5c:	48 8d 7c 24 68       	lea    0x68(%rsp),%rdi
   cde61:	ff 15 49 14 0a 00    	call   *0xa1449(%rip)        # 16f2b0 <_DYNAMIC+0x2a0>
   cde67:	b3 01                	mov    $0x1,%bl
   cde69:	41 b6 01             	mov    $0x1,%r14b
   cde6c:	40 84 ed             	test   %bpl,%bpl
   cde6f:	0f 85 30 02 00 00    	jne    ce0a5 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xfa5>
   cde75:	e9 95 02 00 00       	jmp    ce10f <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x100f>
   cde7a:	49 89 c7             	mov    %rax,%r15
   cde7d:	40 b5 01             	mov    $0x1,%bpl
   cde80:	e9 f2 01 00 00       	jmp    ce077 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xf77>
   cde85:	49 89 c7             	mov    %rax,%r15
   cde88:	40 b5 01             	mov    $0x1,%bpl
   cde8b:	e9 cb 01 00 00       	jmp    ce05b <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xf5b>
   cde90:	49 89 c7             	mov    %rax,%r15
   cde93:	45 31 f6             	xor    %r14d,%r14d
   cde96:	e9 74 01 00 00       	jmp    ce00f <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xf0f>
   cde9b:	49 89 c7             	mov    %rax,%r15
   cde9e:	45 84 f6             	test   %r14b,%r14b
   cdea1:	75 52                	jne    cdef5 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xdf5>
   cdea3:	e9 ac 00 00 00       	jmp    cdf54 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xe54>
   cdea8:	49 89 c7             	mov    %rax,%r15
   cdeab:	eb 19                	jmp    cdec6 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xdc6>
   cdead:	49 89 c7             	mov    %rax,%r15
   cdeb0:	eb 21                	jmp    cded3 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xdd3>
   cdeb2:	49 89 c7             	mov    %rax,%r15
   cdeb5:	f0 49 ff 0e          	lock decq (%r14)
   cdeb9:	75 0b                	jne    cdec6 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xdc6>
   cdebb:	48 8d 7c 24 38       	lea    0x38(%rsp),%rdi
   cdec0:	ff 15 ea 13 0a 00    	call   *0xa13ea(%rip)        # 16f2b0 <_DYNAMIC+0x2a0>
   cdec6:	48 8d bc 24 98 00 00 	lea    0x98(%rsp),%rdi
   cdecd:	00 
   cdece:	e8 dd 29 fe ff       	call   b08b0 <core::ptr::drop_glue::<hyperreal::computable::node::Computable>>
   cded3:	31 db                	xor    %ebx,%ebx
   cded5:	48 8d 7c 24 70       	lea    0x70(%rsp),%rdi
   cdeda:	e8 11 f9 03 00       	call   10d7f0 <<alloc::vec::into_iter::IntoIter<(hyperreal::computable::node::Computable, hyperreal::rational::arithmetic::Rational)> as core::ops::drop::Drop>::drop>
   cdedf:	45 31 f6             	xor    %r14d,%r14d
   cdee2:	e9 28 01 00 00       	jmp    ce00f <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xf0f>
   cdee7:	ff 15 cb 13 0a 00    	call   *0xa13cb(%rip)        # 16f2b8 <_DYNAMIC+0x2a8>
   cdeed:	4c 89 7c 24 58       	mov    %r15,0x58(%rsp)
   cdef2:	49 89 c7             	mov    %rax,%r15
   cdef5:	48 8b 7c 24 08       	mov    0x8(%rsp),%rdi
   cdefa:	e8 21 1b fe ff       	call   afa20 <core::ptr::drop_glue::<alloc::vec::Vec<(hyperreal::rational::arithmetic::Rational, hyperreal::rational::arithmetic::Rational)>>>
   cdeff:	eb 53                	jmp    cdf54 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xe54>
   cdf01:	49 89 c7             	mov    %rax,%r15
   cdf04:	48 8d 7c 24 70       	lea    0x70(%rsp),%rdi
   cdf09:	e8 a2 29 fe ff       	call   b08b0 <core::ptr::drop_glue::<hyperreal::computable::node::Computable>>
   cdf0e:	eb 03                	jmp    cdf13 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xe13>
   cdf10:	49 89 c7             	mov    %rax,%r15
   cdf13:	31 db                	xor    %ebx,%ebx
   cdf15:	45 31 f6             	xor    %r14d,%r14d
   cdf18:	e9 f2 00 00 00       	jmp    ce00f <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xf0f>
   cdf1d:	49 89 c7             	mov    %rax,%r15
   cdf20:	b3 01                	mov    $0x1,%bl
   cdf22:	e9 7e 01 00 00       	jmp    ce0a5 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xfa5>
   cdf27:	49 89 c7             	mov    %rax,%r15
   cdf2a:	eb 77                	jmp    cdfa3 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xea3>
   cdf2c:	4c 89 7c 24 58       	mov    %r15,0x58(%rsp)
   cdf31:	49 89 c7             	mov    %rax,%r15
   cdf34:	48 8b 33             	mov    (%rbx),%rsi
   cdf37:	48 85 f6             	test   %rsi,%rsi
   cdf3a:	74 18                	je     cdf54 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xe54>
   cdf3c:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
   cdf41:	48 8b 78 08          	mov    0x8(%rax),%rdi
   cdf45:	48 c1 e6 04          	shl    $0x4,%rsi
   cdf49:	ba 08 00 00 00       	mov    $0x8,%edx
   cdf4e:	ff 15 7c 13 0a 00    	call   *0xa137c(%rip)        # 16f2d0 <_DYNAMIC+0x2c0>
   cdf54:	48 8b 7c 24 58       	mov    0x58(%rsp),%rdi
   cdf59:	e8 f2 19 fe ff       	call   af950 <core::ptr::drop_glue::<alloc::vec::Vec<(hyperreal::computable::node::Computable, hyperreal::rational::arithmetic::Rational)>>>
   cdf5e:	e9 ac 01 00 00       	jmp    ce10f <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x100f>
   cdf63:	49 89 c7             	mov    %rax,%r15
   cdf66:	f0 48 ff 0b          	lock decq (%rbx)
   cdf6a:	40 b5 01             	mov    $0x1,%bpl
   cdf6d:	0f 85 d2 00 00 00    	jne    ce045 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xf45>
   cdf73:	48 8d bc 24 98 00 00 	lea    0x98(%rsp),%rdi
   cdf7a:	00 
   cdf7b:	ff 15 2f 13 0a 00    	call   *0xa132f(%rip)        # 16f2b0 <_DYNAMIC+0x2a0>
   cdf81:	e9 bf 00 00 00       	jmp    ce045 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xf45>
   cdf86:	49 89 c7             	mov    %rax,%r15
   cdf89:	eb 29                	jmp    cdfb4 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xeb4>
   cdf8b:	49 89 c7             	mov    %rax,%r15
   cdf8e:	f0 49 ff 0c 24       	lock decq (%r12)
   cdf93:	75 0e                	jne    cdfa3 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xea3>
   cdf95:	48 8d bc 24 98 00 00 	lea    0x98(%rsp),%rdi
   cdf9c:	00 
   cdf9d:	ff 15 0d 13 0a 00    	call   *0xa130d(%rip)        # 16f2b0 <_DYNAMIC+0x2a0>
   cdfa3:	f0 48 ff 0b          	lock decq (%rbx)
   cdfa7:	75 0b                	jne    cdfb4 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xeb4>
   cdfa9:	48 8d 7c 24 38       	lea    0x38(%rsp),%rdi
   cdfae:	ff 15 fc 12 0a 00    	call   *0xa12fc(%rip)        # 16f2b0 <_DYNAMIC+0x2a0>
   cdfb4:	31 ed                	xor    %ebp,%ebp
   cdfb6:	e9 8a 00 00 00       	jmp    ce045 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xf45>
   cdfbb:	ff 15 f7 12 0a 00    	call   *0xa12f7(%rip)        # 16f2b8 <_DYNAMIC+0x2a8>
   cdfc1:	eb 1f                	jmp    cdfe2 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xee2>
   cdfc3:	49 89 c7             	mov    %rax,%r15
   cdfc6:	f0 49 ff 0e          	lock decq (%r14)
   cdfca:	b3 01                	mov    $0x1,%bl
   cdfcc:	75 3e                	jne    ce00c <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xf0c>
   cdfce:	b3 01                	mov    $0x1,%bl
   cdfd0:	48 8d 7c 24 70       	lea    0x70(%rsp),%rdi
   cdfd5:	ff 15 d5 12 0a 00    	call   *0xa12d5(%rip)        # 16f2b0 <_DYNAMIC+0x2a0>
   cdfdb:	41 b6 01             	mov    $0x1,%r14b
   cdfde:	eb 2f                	jmp    ce00f <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xf0f>
   cdfe0:	eb 25                	jmp    ce007 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xf07>
   cdfe2:	49 89 c7             	mov    %rax,%r15
   cdfe5:	40 b5 01             	mov    $0x1,%bpl
   cdfe8:	eb 5b                	jmp    ce045 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xf45>
   cdfea:	49 89 c7             	mov    %rax,%r15
   cdfed:	f0 49 ff 0e          	lock decq (%r14)
   cdff1:	b3 01                	mov    $0x1,%bl
   cdff3:	75 17                	jne    ce00c <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xf0c>
   cdff5:	b3 01                	mov    $0x1,%bl
   cdff7:	48 8d 7c 24 70       	lea    0x70(%rsp),%rdi
   cdffc:	ff 15 ae 12 0a 00    	call   *0xa12ae(%rip)        # 16f2b0 <_DYNAMIC+0x2a0>
   ce002:	41 b6 01             	mov    $0x1,%r14b
   ce005:	eb 08                	jmp    ce00f <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xf0f>
   ce007:	49 89 c7             	mov    %rax,%r15
   ce00a:	b3 01                	mov    $0x1,%bl
   ce00c:	41 b6 01             	mov    $0x1,%r14b
   ce00f:	48 8b 84 24 d8 00 00 	mov    0xd8(%rsp),%rax
   ce016:	00 
   ce017:	f0 48 ff 08          	lock decq (%rax)
   ce01b:	75 6e                	jne    ce08b <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xf8b>
   ce01d:	48 8d bc 24 d8 00 00 	lea    0xd8(%rsp),%rdi
   ce024:	00 
   ce025:	ff 15 85 12 0a 00    	call   *0xa1285(%rip)        # 16f2b0 <_DYNAMIC+0x2a0>
   ce02b:	eb 5e                	jmp    ce08b <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xf8b>
   ce02d:	49 89 c7             	mov    %rax,%r15
   ce030:	f0 49 ff 0c 24       	lock decq (%r12)
   ce035:	40 b5 01             	mov    $0x1,%bpl
   ce038:	75 0b                	jne    ce045 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xf45>
   ce03a:	48 8d 7c 24 50       	lea    0x50(%rsp),%rdi
   ce03f:	ff 15 6b 12 0a 00    	call   *0xa126b(%rip)        # 16f2b0 <_DYNAMIC+0x2a0>
   ce045:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
   ce04a:	f0 48 ff 08          	lock decq (%rax)
   ce04e:	75 0b                	jne    ce05b <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xf5b>
   ce050:	48 8d 7c 24 68       	lea    0x68(%rsp),%rdi
   ce055:	ff 15 55 12 0a 00    	call   *0xa1255(%rip)        # 16f2b0 <_DYNAMIC+0x2a0>
   ce05b:	48 8b 84 24 90 00 00 	mov    0x90(%rsp),%rax
   ce062:	00 
   ce063:	f0 48 ff 08          	lock decq (%rax)
   ce067:	75 0e                	jne    ce077 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xf77>
   ce069:	48 8d bc 24 e0 00 00 	lea    0xe0(%rsp),%rdi
   ce070:	00 
   ce071:	ff 15 39 12 0a 00    	call   *0xa1239(%rip)        # 16f2b0 <_DYNAMIC+0x2a0>
   ce077:	48 8d 7c 24 70       	lea    0x70(%rsp),%rdi
   ce07c:	e8 3f f8 03 00       	call   10d8c0 <<alloc::vec::into_iter::IntoIter<(hyperreal::rational::arithmetic::Rational, hyperreal::rational::arithmetic::Rational)> as core::ops::drop::Drop>::drop>
   ce081:	b3 01                	mov    $0x1,%bl
   ce083:	45 31 f6             	xor    %r14d,%r14d
   ce086:	40 84 ed             	test   %bpl,%bpl
   ce089:	75 84                	jne    ce00f <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xf0f>
   ce08b:	48 8d bc 24 a8 00 00 	lea    0xa8(%rsp),%rdi
   ce092:	00 
   ce093:	e8 88 19 fe ff       	call   afa20 <core::ptr::drop_glue::<alloc::vec::Vec<(hyperreal::rational::arithmetic::Rational, hyperreal::rational::arithmetic::Rational)>>>
   ce098:	48 8d bc 24 c0 00 00 	lea    0xc0(%rsp),%rdi
   ce09f:	00 
   ce0a0:	e8 ab 18 fe ff       	call   af950 <core::ptr::drop_glue::<alloc::vec::Vec<(hyperreal::computable::node::Computable, hyperreal::rational::arithmetic::Rational)>>>
   ce0a5:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
   ce0aa:	48 8b 40 30          	mov    0x30(%rax),%rax
   ce0ae:	f0 48 ff 08          	lock decq (%rax)
   ce0b2:	75 0f                	jne    ce0c3 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xfc3>
   ce0b4:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
   ce0b9:	48 8d 78 30          	lea    0x30(%rax),%rdi
   ce0bd:	ff 15 ed 11 0a 00    	call   *0xa11ed(%rip)        # 16f2b0 <_DYNAMIC+0x2a0>
   ce0c3:	45 84 f6             	test   %r14b,%r14b
   ce0c6:	74 0a                	je     ce0d2 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xfd2>
   ce0c8:	48 8b 7c 24 08       	mov    0x8(%rsp),%rdi
   ce0cd:	e8 4e 19 fe ff       	call   afa20 <core::ptr::drop_glue::<alloc::vec::Vec<(hyperreal::rational::arithmetic::Rational, hyperreal::rational::arithmetic::Rational)>>>
   ce0d2:	84 db                	test   %bl,%bl
   ce0d4:	74 39                	je     ce10f <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x100f>
   ce0d6:	48 8b 7c 24 58       	mov    0x58(%rsp),%rdi
   ce0db:	e8 70 18 fe ff       	call   af950 <core::ptr::drop_glue::<alloc::vec::Vec<(hyperreal::computable::node::Computable, hyperreal::rational::arithmetic::Rational)>>>
   ce0e0:	eb 2d                	jmp    ce10f <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x100f>
   ce0e2:	ff 15 d0 11 0a 00    	call   *0xa11d0(%rip)        # 16f2b8 <_DYNAMIC+0x2a8>
   ce0e8:	4d 89 fd             	mov    %r15,%r13
   ce0eb:	49 89 c7             	mov    %rax,%r15
   ce0ee:	66 90                	xchg   %ax,%ax
   ce0f0:	49 ff ce             	dec    %r14
   ce0f3:	74 11                	je     ce106 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x1006>
   ce0f5:	4c 8d 63 18          	lea    0x18(%rbx),%r12
   ce0f9:	48 89 df             	mov    %rbx,%rdi
   ce0fc:	e8 bf 36 04 00       	call   1117c0 <core::ptr::drop_glue::<(hyperreal::computable::node::Computable, hyperreal::rational::arithmetic::Rational)>>
   ce101:	4c 89 e3             	mov    %r12,%rbx
   ce104:	eb ea                	jmp    ce0f0 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xff0>
   ce106:	49 8b 45 00          	mov    0x0(%r13),%rax
   ce10a:	48 85 c0             	test   %rax,%rax
   ce10d:	75 08                	jne    ce117 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x1017>
   ce10f:	4c 89 ff             	mov    %r15,%rdi
   ce112:	e8 69 ae 09 00       	call   168f80 <_Unwind_Resume@plt>
   ce117:	48 8b 4c 24 08       	mov    0x8(%rsp),%rcx
   ce11c:	48 8b 79 20          	mov    0x20(%rcx),%rdi
   ce120:	48 c1 e0 03          	shl    $0x3,%rax
   ce124:	48 8d 34 40          	lea    (%rax,%rax,2),%rsi
   ce128:	ba 08 00 00 00       	mov    $0x8,%edx
   ce12d:	ff 15 9d 11 0a 00    	call   *0xa119d(%rip)        # 16f2d0 <_DYNAMIC+0x2c0>
   ce133:	4c 89 ff             	mov    %r15,%rdi
   ce136:	e8 45 ae 09 00       	call   168f80 <_Unwind_Resume@plt>
   ce13b:	ff 15 77 11 0a 00    	call   *0xa1177(%rip)        # 16f2b8 <_DYNAMIC+0x2a8>
