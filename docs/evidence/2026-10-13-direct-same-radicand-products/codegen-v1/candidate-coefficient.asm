
/home/tim/Documents/GitHub/workspace/csgrs/target/direct-same-radicand-products-v2/cost-v2-binaries/candidate-quadratic-time:     file format elf64-x86-64


Disassembly of section .text:

00000000000cd0f0 <<hyperreal::computable::node::RadicalCoefficient>::multiply>:
   cd0f0:	55                   	push   %rbp
   cd0f1:	41 57                	push   %r15
   cd0f3:	41 56                	push   %r14
   cd0f5:	41 55                	push   %r13
   cd0f7:	41 54                	push   %r12
   cd0f9:	53                   	push   %rbx
   cd0fa:	48 81 ec 08 01 00 00 	sub    $0x108,%rsp
   cd101:	4c 8d 7e 18          	lea    0x18(%rsi),%r15
   cd105:	48 89 74 24 08       	mov    %rsi,0x8(%rsp)
   cd10a:	48 83 7e 28 00       	cmpq   $0x0,0x28(%rsi)
   cd10f:	74 42                	je     cd153 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x63>
   cd111:	48 83 7a 10 00       	cmpq   $0x0,0x10(%rdx)
   cd116:	75 54                	jne    cd16c <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x7c>
   cd118:	48 83 7a 28 00       	cmpq   $0x0,0x28(%rdx)
   cd11d:	75 4d                	jne    cd16c <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x7c>
   cd11f:	48 89 7c 24 40       	mov    %rdi,0x40(%rsp)
   cd124:	4c 89 7c 24 58       	mov    %r15,0x58(%rsp)
   cd129:	48 8b 7c 24 08       	mov    0x8(%rsp),%rdi
   cd12e:	48 8d 6f 10          	lea    0x10(%rdi),%rbp
   cd132:	4c 8d 6f 08          	lea    0x8(%rdi),%r13
   cd136:	48 8b 42 08          	mov    0x8(%rdx),%rax
   cd13a:	48 89 44 24 60       	mov    %rax,0x60(%rsp)
   cd13f:	b0 01                	mov    $0x1,%al
   cd141:	89 44 24 2c          	mov    %eax,0x2c(%rsp)
   cd145:	48 c7 44 24 18 00 00 	movq   $0x0,0x18(%rsp)
   cd14c:	00 00 
   cd14e:	e9 0e 02 00 00       	jmp    cd361 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x271>
   cd153:	48 8b 42 28          	mov    0x28(%rdx),%rax
   cd157:	48 85 c0             	test   %rax,%rax
   cd15a:	74 39                	je     cd195 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xa5>
   cd15c:	48 8b 4c 24 08       	mov    0x8(%rsp),%rcx
   cd161:	48 83 79 10 00       	cmpq   $0x0,0x10(%rcx)
   cd166:	0f 84 af 01 00 00    	je     cd31b <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x22b>
   cd16c:	48 c7 07 ff ff ff ff 	movq   $0xffffffffffffffff,(%rdi)
   cd173:	48 8b 5c 24 08       	mov    0x8(%rsp),%rbx
   cd178:	48 8b 43 30          	mov    0x30(%rbx),%rax
   cd17c:	f0 48 ff 08          	lock decq (%rax)
   cd180:	0f 85 5d 03 00 00    	jne    cd4e3 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x3f3>
   cd186:	48 8d 7b 30          	lea    0x30(%rbx),%rdi
   cd18a:	ff 15 10 21 0a 00    	call   *0xa2110(%rip)        # 16f2a0 <_DYNAMIC+0x2a0>
   cd190:	e9 4e 03 00 00       	jmp    cd4e3 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x3f3>
   cd195:	48 89 7c 24 40       	mov    %rdi,0x40(%rsp)
   cd19a:	4c 89 7c 24 58       	mov    %r15,0x58(%rsp)
   cd19f:	48 8b 72 08          	mov    0x8(%rdx),%rsi
   cd1a3:	48 8b 4a 10          	mov    0x10(%rdx),%rcx
   cd1a7:	48 8b 7c 24 08       	mov    0x8(%rsp),%rdi
   cd1ac:	48 8d 6f 10          	lea    0x10(%rdi),%rbp
   cd1b0:	48 8b 47 10          	mov    0x10(%rdi),%rax
   cd1b4:	48 83 f0 01          	xor    $0x1,%rax
   cd1b8:	4c 8d 6f 08          	lea    0x8(%rdi),%r13
   cd1bc:	48 89 4c 24 18       	mov    %rcx,0x18(%rsp)
   cd1c1:	48 83 f1 01          	xor    $0x1,%rcx
   cd1c5:	48 09 c1             	or     %rax,%rcx
   cd1c8:	48 89 74 24 60       	mov    %rsi,0x60(%rsp)
   cd1cd:	0f 85 88 01 00 00    	jne    cd35b <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x26b>
   cd1d3:	49 89 d4             	mov    %rdx,%r12
   cd1d6:	48 8b 5f 08          	mov    0x8(%rdi),%rbx
   cd1da:	41 b6 01             	mov    $0x1,%r14b
   cd1dd:	48 89 df             	mov    %rbx,%rdi
   cd1e0:	ff 15 62 20 0a 00    	call   *0xa2062(%rip)        # 16f248 <_DYNAMIC+0x248>
   cd1e6:	b9 01 00 00 00       	mov    $0x1,%ecx
   cd1eb:	48 89 4c 24 18       	mov    %rcx,0x18(%rsp)
   cd1f0:	84 c0                	test   %al,%al
   cd1f2:	0f 84 8e 05 00 00    	je     cd786 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x696>
   cd1f8:	4c 8b 3b             	mov    (%rbx),%r15
   cd1fb:	41 8b 47 60          	mov    0x60(%r15),%eax
   cd1ff:	84 c0                	test   %al,%al
   cd201:	0f 88 41 07 00 00    	js     cd948 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x858>
   cd207:	41 80 7f 64 02       	cmpb   $0x2,0x64(%r15)
   cd20c:	75 63                	jne    cd271 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x181>
   cd20e:	4c 8b 35 73 20 0a 00 	mov    0xa2073(%rip),%r14        # 16f288 <_DYNAMIC+0x288>
   cd215:	41 8b 46 18          	mov    0x18(%r14),%eax
   cd219:	85 c0                	test   %eax,%eax
   cd21b:	0f 85 bd 09 00 00    	jne    cdbde <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xaee>
   cd221:	49 8b 57 20          	mov    0x20(%r15),%rdx
   cd225:	49 3b 56 10          	cmp    0x10(%r14),%rdx
   cd229:	75 46                	jne    cd271 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x181>
   cd22b:	49 8b 76 08          	mov    0x8(%r14),%rsi
   cd22f:	49 8b 7f 18          	mov    0x18(%r15),%rdi
   cd233:	48 c1 e2 03          	shl    $0x3,%rdx
   cd237:	ff 15 53 20 0a 00    	call   *0xa2053(%rip)        # 16f290 <bcmp@GLIBC_2.2.5>
   cd23d:	85 c0                	test   %eax,%eax
   cd23f:	75 30                	jne    cd271 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x181>
   cd241:	41 8b 46 18          	mov    0x18(%r14),%eax
   cd245:	85 c0                	test   %eax,%eax
   cd247:	0f 85 e8 09 00 00    	jne    cdc35 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xb45>
   cd24d:	49 8b 57 38          	mov    0x38(%r15),%rdx
   cd251:	49 3b 56 10          	cmp    0x10(%r14),%rdx
   cd255:	75 1a                	jne    cd271 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x181>
   cd257:	49 8b 76 08          	mov    0x8(%r14),%rsi
   cd25b:	49 8b 7f 30          	mov    0x30(%r15),%rdi
   cd25f:	48 c1 e2 03          	shl    $0x3,%rdx
   cd263:	ff 15 27 20 0a 00    	call   *0xa2027(%rip)        # 16f290 <bcmp@GLIBC_2.2.5>
   cd269:	85 c0                	test   %eax,%eax
   cd26b:	0f 84 15 05 00 00    	je     cd786 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x696>
   cd271:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
   cd276:	48 8d 78 30          	lea    0x30(%rax),%rdi
   cd27a:	49 83 c4 30          	add    $0x30,%r12
   cd27e:	41 b6 01             	mov    $0x1,%r14b
   cd281:	48 89 7c 24 30       	mov    %rdi,0x30(%rsp)
   cd286:	4d 89 e7             	mov    %r12,%r15
   cd289:	4c 89 e6             	mov    %r12,%rsi
   cd28c:	e8 af 44 fc ff       	call   91740 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cd291:	48 8d 7b 08          	lea    0x8(%rbx),%rdi
   cd295:	48 89 44 24 48       	mov    %rax,0x48(%rsp)
   cd29a:	48 89 44 24 68       	mov    %rax,0x68(%rsp)
   cd29f:	48 89 bc 24 90 00 00 	mov    %rdi,0x90(%rsp)
   cd2a6:	00 
   cd2a7:	4c 89 fe             	mov    %r15,%rsi
   cd2aa:	e8 91 44 fc ff       	call   91740 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cd2af:	49 89 c4             	mov    %rax,%r12
   cd2b2:	48 8b 44 24 60       	mov    0x60(%rsp),%rax
   cd2b7:	48 8d 78 08          	lea    0x8(%rax),%rdi
   cd2bb:	4c 89 64 24 50       	mov    %r12,0x50(%rsp)
   cd2c0:	b0 01                	mov    $0x1,%al
   cd2c2:	89 44 24 10          	mov    %eax,0x10(%rsp)
   cd2c6:	41 b7 01             	mov    $0x1,%r15b
   cd2c9:	49 89 fe             	mov    %rdi,%r14
   cd2cc:	48 8b 74 24 30       	mov    0x30(%rsp),%rsi
   cd2d1:	e8 6a 44 fc ff       	call   91740 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cd2d6:	48 89 44 24 38       	mov    %rax,0x38(%rsp)
   cd2db:	48 89 c6             	mov    %rax,%rsi
   cd2de:	4c 89 e0             	mov    %r12,%rax
   cd2e1:	45 0f b6 64 24 64    	movzbl 0x64(%r12),%r12d
   cd2e7:	41 80 fc 01          	cmp    $0x1,%r12b
   cd2eb:	48 89 74 24 18       	mov    %rsi,0x18(%rsp)
   cd2f0:	49 89 df             	mov    %rbx,%r15
   cd2f3:	0f 85 8a 06 00 00    	jne    cd983 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x893>
   cd2f9:	f0 48 ff 08          	lock decq (%rax)
   cd2fd:	48 89 f0             	mov    %rsi,%rax
   cd300:	0f 85 a4 06 00 00    	jne    cd9aa <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x8ba>
   cd306:	48 8d 7c 24 50       	lea    0x50(%rsp),%rdi
   cd30b:	ff 15 8f 1f 0a 00    	call   *0xa1f8f(%rip)        # 16f2a0 <_DYNAMIC+0x2a0>
   cd311:	48 8b 44 24 18       	mov    0x18(%rsp),%rax
   cd316:	e9 8f 06 00 00       	jmp    cd9aa <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x8ba>
   cd31b:	48 89 7c 24 40       	mov    %rdi,0x40(%rsp)
   cd320:	4c 89 7c 24 58       	mov    %r15,0x58(%rsp)
   cd325:	48 8b 7c 24 08       	mov    0x8(%rsp),%rdi
   cd32a:	48 8d 6f 10          	lea    0x10(%rdi),%rbp
   cd32e:	4c 8d 6f 08          	lea    0x8(%rdi),%r13
   cd332:	48 8b 4a 08          	mov    0x8(%rdx),%rcx
   cd336:	48 89 4c 24 60       	mov    %rcx,0x60(%rsp)
   cd33b:	48 8b 4a 10          	mov    0x10(%rdx),%rcx
   cd33f:	48 89 4c 24 18       	mov    %rcx,0x18(%rsp)
   cd344:	48 c1 e0 03          	shl    $0x3,%rax
   cd348:	48 8d 04 40          	lea    (%rax,%rax,2),%rax
   cd34c:	48 89 44 24 30       	mov    %rax,0x30(%rsp)
   cd351:	c7 44 24 2c 00 00 00 	movl   $0x0,0x2c(%rsp)
   cd358:	00 
   cd359:	eb 0f                	jmp    cd36a <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x27a>
   cd35b:	b0 01                	mov    $0x1,%al
   cd35d:	89 44 24 2c          	mov    %eax,0x2c(%rsp)
   cd361:	48 c7 44 24 30 00 00 	movq   $0x0,0x30(%rsp)
   cd368:	00 00 
   cd36a:	48 83 c7 30          	add    $0x30,%rdi
   cd36e:	48 89 94 24 f0 00 00 	mov    %rdx,0xf0(%rsp)
   cd375:	00 
   cd376:	48 8d 72 30          	lea    0x30(%rdx),%rsi
   cd37a:	41 b6 01             	mov    $0x1,%r14b
   cd37d:	48 89 7c 24 48       	mov    %rdi,0x48(%rsp)
   cd382:	48 89 74 24 20       	mov    %rsi,0x20(%rsp)
   cd387:	e8 b4 43 fc ff       	call   91740 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cd38c:	48 89 84 24 d8 00 00 	mov    %rax,0xd8(%rsp)
   cd393:	00 
   cd394:	48 c7 84 24 a8 00 00 	movq   $0x0,0xa8(%rsp)
   cd39b:	00 00 00 00 00 
   cd3a0:	48 c7 84 24 b0 00 00 	movq   $0x8,0xb0(%rsp)
   cd3a7:	00 08 00 00 00 
   cd3ac:	0f 57 c0             	xorps  %xmm0,%xmm0
   cd3af:	0f 11 84 24 b8 00 00 	movups %xmm0,0xb8(%rsp)
   cd3b6:	00 
   cd3b7:	48 c7 84 24 c8 00 00 	movq   $0x8,0xc8(%rsp)
   cd3be:	00 08 00 00 00 
   cd3c3:	48 c7 84 24 d0 00 00 	movq   $0x0,0xd0(%rsp)
   cd3ca:	00 00 00 00 00 
   cd3cf:	4d 8b 7d 00          	mov    0x0(%r13),%r15
   cd3d3:	4c 8b 65 00          	mov    0x0(%rbp),%r12
   cd3d7:	49 c1 e4 04          	shl    $0x4,%r12
   cd3db:	48 8d 9c 24 a8 00 00 	lea    0xa8(%rsp),%rbx
   cd3e2:	00 
   cd3e3:	4c 8b 6c 24 40       	mov    0x40(%rsp),%r13
   cd3e8:	0f 1f 84 00 00 00 00 	nopl   0x0(%rax,%rax,1)
   cd3ef:	00 
   cd3f0:	4d 85 e4             	test   %r12,%r12
   cd3f3:	74 3f                	je     cd434 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x344>
   cd3f5:	49 8b 07             	mov    (%r15),%rax
   cd3f8:	f0 48 ff 00          	lock incq (%rax)
   cd3fc:	0f 8e ae 08 00 00    	jle    cdcb0 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xbc0>
   cd402:	49 8d 7f 08          	lea    0x8(%r15),%rdi
   cd406:	4d 8b 37             	mov    (%r15),%r14
   cd409:	4c 89 74 24 70       	mov    %r14,0x70(%rsp)
   cd40e:	48 8b 74 24 20       	mov    0x20(%rsp),%rsi
   cd413:	e8 28 43 fc ff       	call   91740 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cd418:	48 89 df             	mov    %rbx,%rdi
   cd41b:	4c 89 f6             	mov    %r14,%rsi
   cd41e:	48 89 c2             	mov    %rax,%rdx
   cd421:	e8 1a f4 ff ff       	call   cc840 <<hyperreal::computable::node::RadicalCoefficient>::insert_radical>
   cd426:	49 83 c7 10          	add    $0x10,%r15
   cd42a:	49 83 c4 f0          	add    $0xfffffffffffffff0,%r12
   cd42e:	84 c0                	test   %al,%al
   cd430:	75 be                	jne    cd3f0 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x300>
   cd432:	eb 75                	jmp    cd4a9 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x3b9>
   cd434:	4c 8b 7c 24 18       	mov    0x18(%rsp),%r15
   cd439:	49 c1 e7 04          	shl    $0x4,%r15
   cd43d:	48 8d 9c 24 a8 00 00 	lea    0xa8(%rsp),%rbx
   cd444:	00 
   cd445:	4c 89 bc 24 f8 00 00 	mov    %r15,0xf8(%rsp)
   cd44c:	00 
   cd44d:	4c 8b 64 24 60       	mov    0x60(%rsp),%r12
   cd452:	66 66 66 66 66 2e 0f 	data16 data16 data16 data16 cs nopw 0x0(%rax,%rax,1)
   cd459:	1f 84 00 00 00 00 00 
   cd460:	4d 85 ff             	test   %r15,%r15
   cd463:	0f 84 04 01 00 00    	je     cd56d <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x47d>
   cd469:	49 8b 04 24          	mov    (%r12),%rax
   cd46d:	f0 48 ff 00          	lock incq (%rax)
   cd471:	0f 8e 39 08 00 00    	jle    cdcb0 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xbc0>
   cd477:	49 8d 7c 24 08       	lea    0x8(%r12),%rdi
   cd47c:	4d 8b 34 24          	mov    (%r12),%r14
   cd480:	4c 89 74 24 70       	mov    %r14,0x70(%rsp)
   cd485:	48 8b 74 24 48       	mov    0x48(%rsp),%rsi
   cd48a:	e8 b1 42 fc ff       	call   91740 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cd48f:	48 89 df             	mov    %rbx,%rdi
   cd492:	4c 89 f6             	mov    %r14,%rsi
   cd495:	48 89 c2             	mov    %rax,%rdx
   cd498:	e8 a3 f3 ff ff       	call   cc840 <<hyperreal::computable::node::RadicalCoefficient>::insert_radical>
   cd49d:	49 83 c4 10          	add    $0x10,%r12
   cd4a1:	49 83 c7 f0          	add    $0xfffffffffffffff0,%r15
   cd4a5:	84 c0                	test   %al,%al
   cd4a7:	75 b7                	jne    cd460 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x370>
   cd4a9:	49 c7 45 00 ff ff ff 	movq   $0xffffffffffffffff,0x0(%r13)
   cd4b0:	ff 
   cd4b1:	41 b6 01             	mov    $0x1,%r14b
   cd4b4:	48 8d bc 24 a8 00 00 	lea    0xa8(%rsp),%rdi
   cd4bb:	00 
   cd4bc:	e8 7f 38 fe ff       	call   b0d40 <core::ptr::drop_glue::<hyperreal::computable::node::RadicalCoefficient>>
   cd4c1:	48 8b 5c 24 08       	mov    0x8(%rsp),%rbx
   cd4c6:	48 8b 43 30          	mov    0x30(%rbx),%rax
   cd4ca:	f0 48 ff 08          	lock decq (%rax)
   cd4ce:	75 0a                	jne    cd4da <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x3ea>
   cd4d0:	48 8d 7b 30          	lea    0x30(%rbx),%rdi
   cd4d4:	ff 15 c6 1d 0a 00    	call   *0xa1dc6(%rip)        # 16f2a0 <_DYNAMIC+0x2a0>
   cd4da:	45 84 f6             	test   %r14b,%r14b
   cd4dd:	4c 8d 7b 18          	lea    0x18(%rbx),%r15
   cd4e1:	74 23                	je     cd506 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x416>
   cd4e3:	48 89 df             	mov    %rbx,%rdi
   cd4e6:	e8 35 5e 04 00       	call   113320 <<alloc::vec::Vec<(hyperreal::rational::arithmetic::Rational, hyperreal::rational::arithmetic::Rational)> as core::ops::drop::Drop>::drop>
   cd4eb:	48 8b 33             	mov    (%rbx),%rsi
   cd4ee:	48 85 f6             	test   %rsi,%rsi
   cd4f1:	74 13                	je     cd506 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x416>
   cd4f3:	48 8b 7b 08          	mov    0x8(%rbx),%rdi
   cd4f7:	48 c1 e6 04          	shl    $0x4,%rsi
   cd4fb:	ba 08 00 00 00       	mov    $0x8,%edx
   cd500:	ff 15 ba 1d 0a 00    	call   *0xa1dba(%rip)        # 16f2c0 <_DYNAMIC+0x2c0>
   cd506:	48 8b 7b 20          	mov    0x20(%rbx),%rdi
   cd50a:	4c 8b 73 28          	mov    0x28(%rbx),%r14
   cd50e:	49 ff c6             	inc    %r14
   cd511:	66 66 66 66 66 66 2e 	data16 data16 data16 data16 data16 cs nopw 0x0(%rax,%rax,1)
   cd518:	0f 1f 84 00 00 00 00 
   cd51f:	00 
   cd520:	49 83 fe 01          	cmp    $0x1,%r14
   cd524:	74 11                	je     cd537 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x447>
   cd526:	48 8d 5f 18          	lea    0x18(%rdi),%rbx
   cd52a:	49 ff ce             	dec    %r14
   cd52d:	e8 7e 42 04 00       	call   1117b0 <core::ptr::drop_glue::<(hyperreal::computable::node::Computable, hyperreal::rational::arithmetic::Rational)>>
   cd532:	48 89 df             	mov    %rbx,%rdi
   cd535:	eb e9                	jmp    cd520 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x430>
   cd537:	49 8b 07             	mov    (%r15),%rax
   cd53a:	48 85 c0             	test   %rax,%rax
   cd53d:	74 1c                	je     cd55b <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x46b>
   cd53f:	48 8b 4c 24 08       	mov    0x8(%rsp),%rcx
   cd544:	48 8b 79 20          	mov    0x20(%rcx),%rdi
   cd548:	48 c1 e0 03          	shl    $0x3,%rax
   cd54c:	48 8d 34 40          	lea    (%rax,%rax,2),%rsi
   cd550:	ba 08 00 00 00       	mov    $0x8,%edx
   cd555:	ff 15 65 1d 0a 00    	call   *0xa1d65(%rip)        # 16f2c0 <_DYNAMIC+0x2c0>
   cd55b:	48 81 c4 08 01 00 00 	add    $0x108,%rsp
   cd562:	5b                   	pop    %rbx
   cd563:	41 5c                	pop    %r12
   cd565:	41 5d                	pop    %r13
   cd567:	41 5e                	pop    %r14
   cd569:	41 5f                	pop    %r15
   cd56b:	5d                   	pop    %rbp
   cd56c:	c3                   	ret
   cd56d:	4c 89 6c 24 40       	mov    %r13,0x40(%rsp)
   cd572:	48 8b 4c 24 08       	mov    0x8(%rsp),%rcx
   cd577:	48 8b 01             	mov    (%rcx),%rax
   cd57a:	48 8b 59 08          	mov    0x8(%rcx),%rbx
   cd57e:	48 8b 49 10          	mov    0x10(%rcx),%rcx
   cd582:	48 89 ca             	mov    %rcx,%rdx
   cd585:	48 c1 e2 04          	shl    $0x4,%rdx
   cd589:	48 01 da             	add    %rbx,%rdx
   cd58c:	48 89 5c 24 70       	mov    %rbx,0x70(%rsp)
   cd591:	48 89 5c 24 78       	mov    %rbx,0x78(%rsp)
   cd596:	48 89 84 24 80 00 00 	mov    %rax,0x80(%rsp)
   cd59d:	00 
   cd59e:	48 89 94 24 e8 00 00 	mov    %rdx,0xe8(%rsp)
   cd5a5:	00 
   cd5a6:	48 89 94 24 88 00 00 	mov    %rdx,0x88(%rsp)
   cd5ad:	00 
   cd5ae:	48 85 c9             	test   %rcx,%rcx
   cd5b1:	0f 84 eb 01 00 00    	je     cd7a2 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x6b2>
   cd5b7:	4c 8d bc 24 e0 00 00 	lea    0xe0(%rsp),%r15
   cd5be:	00 
   cd5bf:	eb 0e                	jmp    cd5cf <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x4df>
   cd5c1:	48 3b 9c 24 e8 00 00 	cmp    0xe8(%rsp),%rbx
   cd5c8:	00 
   cd5c9:	0f 84 d3 01 00 00    	je     cd7a2 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x6b2>
   cd5cf:	48 8d 4b 10          	lea    0x10(%rbx),%rcx
   cd5d3:	48 89 8c 24 00 01 00 	mov    %rcx,0x100(%rsp)
   cd5da:	00 
   cd5db:	48 89 4c 24 78       	mov    %rcx,0x78(%rsp)
   cd5e0:	48 8b 0b             	mov    (%rbx),%rcx
   cd5e3:	48 8b 43 08          	mov    0x8(%rbx),%rax
   cd5e7:	48 89 8c 24 90 00 00 	mov    %rcx,0x90(%rsp)
   cd5ee:	00 
   cd5ef:	48 89 8c 24 e0 00 00 	mov    %rcx,0xe0(%rsp)
   cd5f6:	00 
   cd5f7:	48 89 44 24 10       	mov    %rax,0x10(%rsp)
   cd5fc:	48 89 44 24 68       	mov    %rax,0x68(%rsp)
   cd601:	48 8b ac 24 f8 00 00 	mov    0xf8(%rsp),%rbp
   cd608:	00 
   cd609:	4c 8b 74 24 60       	mov    0x60(%rsp),%r14
   cd60e:	48 83 7c 24 18 00    	cmpq   $0x0,0x18(%rsp)
   cd614:	75 67                	jne    cd67d <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x58d>
   cd616:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
   cd61b:	f0 48 ff 08          	lock decq (%rax)
   cd61f:	75 0b                	jne    cd62c <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x53c>
   cd621:	48 8d 7c 24 68       	lea    0x68(%rsp),%rdi
   cd626:	ff 15 74 1c 0a 00    	call   *0xa1c74(%rip)        # 16f2a0 <_DYNAMIC+0x2a0>
   cd62c:	48 8b 84 24 90 00 00 	mov    0x90(%rsp),%rax
   cd633:	00 
   cd634:	f0 48 ff 08          	lock decq (%rax)
   cd638:	48 8b 9c 24 00 01 00 	mov    0x100(%rsp),%rbx
   cd63f:	00 
   cd640:	0f 85 7b ff ff ff    	jne    cd5c1 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x4d1>
   cd646:	4c 89 ff             	mov    %r15,%rdi
   cd649:	ff 15 51 1c 0a 00    	call   *0xa1c51(%rip)        # 16f2a0 <_DYNAMIC+0x2a0>
   cd64f:	e9 6d ff ff ff       	jmp    cd5c1 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x4d1>
   cd654:	66 66 66 2e 0f 1f 84 	data16 data16 cs nopw 0x0(%rax,%rax,1)
   cd65b:	00 00 00 00 00 
   cd660:	48 89 df             	mov    %rbx,%rdi
   cd663:	4c 89 ee             	mov    %r13,%rsi
   cd666:	e8 a5 27 fc ff       	call   8fe10 <<hyperreal::rational::arithmetic::Rational as core::ops::arith::Add>::add>
   cd66b:	48 89 84 24 d8 00 00 	mov    %rax,0xd8(%rsp)
   cd672:	00 
   cd673:	49 83 c6 10          	add    $0x10,%r14
   cd677:	48 83 c5 f0          	add    $0xfffffffffffffff0,%rbp
   cd67b:	74 99                	je     cd616 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x526>
   cd67d:	49 8d 76 08          	lea    0x8(%r14),%rsi
   cd681:	48 8d 7c 24 68       	lea    0x68(%rsp),%rdi
   cd686:	e8 b5 40 fc ff       	call   91740 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cd68b:	49 89 c4             	mov    %rax,%r12
   cd68e:	48 89 44 24 50       	mov    %rax,0x50(%rsp)
   cd693:	4c 89 ff             	mov    %r15,%rdi
   cd696:	4c 89 f6             	mov    %r14,%rsi
   cd699:	ff 15 a9 1b 0a 00    	call   *0xa1ba9(%rip)        # 16f248 <_DYNAMIC+0x248>
   cd69f:	84 c0                	test   %al,%al
   cd6a1:	74 4d                	je     cd6f0 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x600>
   cd6a3:	48 8b 9c 24 d8 00 00 	mov    0xd8(%rsp),%rbx
   cd6aa:	00 
   cd6ab:	48 89 5c 24 38       	mov    %rbx,0x38(%rsp)
   cd6b0:	4c 89 a4 24 98 00 00 	mov    %r12,0x98(%rsp)
   cd6b7:	00 
   cd6b8:	48 8d bc 24 98 00 00 	lea    0x98(%rsp),%rdi
   cd6bf:	00 
   cd6c0:	4c 89 fe             	mov    %r15,%rsi
   cd6c3:	e8 78 40 fc ff       	call   91740 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cd6c8:	49 89 c5             	mov    %rax,%r13
   cd6cb:	f0 49 ff 0c 24       	lock decq (%r12)
   cd6d0:	75 8e                	jne    cd660 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x570>
   cd6d2:	48 8d bc 24 98 00 00 	lea    0x98(%rsp),%rdi
   cd6d9:	00 
   cd6da:	ff 15 c0 1b 0a 00    	call   *0xa1bc0(%rip)        # 16f2a0 <_DYNAMIC+0x2a0>
   cd6e0:	e9 7b ff ff ff       	jmp    cd660 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x570>
   cd6e5:	66 66 2e 0f 1f 84 00 	data16 cs nopw 0x0(%rax,%rax,1)
   cd6ec:	00 00 00 00 
   cd6f0:	4c 89 ff             	mov    %r15,%rdi
   cd6f3:	4c 89 f6             	mov    %r14,%rsi
   cd6f6:	e8 45 40 fc ff       	call   91740 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cd6fb:	48 89 c7             	mov    %rax,%rdi
   cd6fe:	e8 0d 5c fa ff       	call   73310 <<hyperreal::rational::arithmetic::Rational>::extract_square_reduced_retained>
   cd703:	48 89 d3             	mov    %rdx,%rbx
   cd706:	48 89 94 24 98 00 00 	mov    %rdx,0x98(%rsp)
   cd70d:	00 
   cd70e:	4c 89 e7             	mov    %r12,%rdi
   cd711:	48 89 c6             	mov    %rax,%rsi
   cd714:	e8 a7 4b fc ff       	call   922c0 <<hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cd719:	48 8d bc 24 a8 00 00 	lea    0xa8(%rsp),%rdi
   cd720:	00 
   cd721:	48 89 de             	mov    %rbx,%rsi
   cd724:	48 89 c2             	mov    %rax,%rdx
   cd727:	e8 14 f1 ff ff       	call   cc840 <<hyperreal::computable::node::RadicalCoefficient>::insert_radical>
   cd72c:	84 c0                	test   %al,%al
   cd72e:	0f 85 3f ff ff ff    	jne    cd673 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x583>
   cd734:	48 8b 44 24 40       	mov    0x40(%rsp),%rax
   cd739:	48 c7 00 ff ff ff ff 	movq   $0xffffffffffffffff,(%rax)
   cd740:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
   cd745:	f0 48 ff 08          	lock decq (%rax)
   cd749:	75 0b                	jne    cd756 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x666>
   cd74b:	48 8d 7c 24 68       	lea    0x68(%rsp),%rdi
   cd750:	ff 15 4a 1b 0a 00    	call   *0xa1b4a(%rip)        # 16f2a0 <_DYNAMIC+0x2a0>
   cd756:	48 8b 84 24 90 00 00 	mov    0x90(%rsp),%rax
   cd75d:	00 
   cd75e:	f0 48 ff 08          	lock decq (%rax)
   cd762:	75 0e                	jne    cd772 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x682>
   cd764:	48 8d bc 24 e0 00 00 	lea    0xe0(%rsp),%rdi
   cd76b:	00 
   cd76c:	ff 15 2e 1b 0a 00    	call   *0xa1b2e(%rip)        # 16f2a0 <_DYNAMIC+0x2a0>
   cd772:	b3 01                	mov    $0x1,%bl
   cd774:	48 8d 7c 24 70       	lea    0x70(%rsp),%rdi
   cd779:	e8 32 01 04 00       	call   10d8b0 <<alloc::vec::into_iter::IntoIter<(hyperreal::rational::arithmetic::Rational, hyperreal::rational::arithmetic::Rational)> as core::ops::drop::Drop>::drop>
   cd77e:	45 31 f6             	xor    %r14d,%r14d
   cd781:	e9 2e fd ff ff       	jmp    cd4b4 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x3c4>
   cd786:	b0 01                	mov    $0x1,%al
   cd788:	89 44 24 2c          	mov    %eax,0x2c(%rsp)
   cd78c:	48 c7 44 24 30 00 00 	movq   $0x0,0x30(%rsp)
   cd793:	00 00 
   cd795:	48 8b 7c 24 08       	mov    0x8(%rsp),%rdi
   cd79a:	4c 89 e2             	mov    %r12,%rdx
   cd79d:	e9 c8 fb ff ff       	jmp    cd36a <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x27a>
   cd7a2:	b3 01                	mov    $0x1,%bl
   cd7a4:	48 8d 7c 24 70       	lea    0x70(%rsp),%rdi
   cd7a9:	e8 02 01 04 00       	call   10d8b0 <<alloc::vec::into_iter::IntoIter<(hyperreal::rational::arithmetic::Rational, hyperreal::rational::arithmetic::Rational)> as core::ops::drop::Drop>::drop>
   cd7ae:	48 8b 4c 24 08       	mov    0x8(%rsp),%rcx
   cd7b3:	48 8b 41 18          	mov    0x18(%rcx),%rax
   cd7b7:	4c 8b 69 20          	mov    0x20(%rcx),%r13
   cd7bb:	48 8b 49 28          	mov    0x28(%rcx),%rcx
   cd7bf:	48 8d 14 cd 00 00 00 	lea    0x0(,%rcx,8),%rdx
   cd7c6:	00 
   cd7c7:	4c 8d 3c 52          	lea    (%rdx,%rdx,2),%r15
   cd7cb:	4b 8d 54 3d 00       	lea    0x0(%r13,%r15,1),%rdx
   cd7d0:	4c 89 6c 24 70       	mov    %r13,0x70(%rsp)
   cd7d5:	4c 89 6c 24 78       	mov    %r13,0x78(%rsp)
   cd7da:	48 89 84 24 80 00 00 	mov    %rax,0x80(%rsp)
   cd7e1:	00 
   cd7e2:	48 89 94 24 88 00 00 	mov    %rdx,0x88(%rsp)
   cd7e9:	00 
   cd7ea:	48 85 c9             	test   %rcx,%rcx
   cd7ed:	74 7f                	je     cd86e <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x77e>
   cd7ef:	49 83 c5 18          	add    $0x18,%r13
   cd7f3:	66 66 66 66 2e 0f 1f 	data16 data16 data16 cs nopw 0x0(%rax,%rax,1)
   cd7fa:	84 00 00 00 00 00 
   cd800:	4c 89 6c 24 78       	mov    %r13,0x78(%rsp)
   cd805:	49 8b 5d e8          	mov    -0x18(%r13),%rbx
   cd809:	48 85 db             	test   %rbx,%rbx
   cd80c:	74 60                	je     cd86e <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x77e>
   cd80e:	4d 8b 65 f0          	mov    -0x10(%r13),%r12
   cd812:	4d 8b 75 f8          	mov    -0x8(%r13),%r14
   cd816:	48 89 9c 24 98 00 00 	mov    %rbx,0x98(%rsp)
   cd81d:	00 
   cd81e:	4c 89 a4 24 a0 00 00 	mov    %r12,0xa0(%rsp)
   cd825:	00 
   cd826:	4c 89 74 24 38       	mov    %r14,0x38(%rsp)
   cd82b:	48 8d 7c 24 38       	lea    0x38(%rsp),%rdi
   cd830:	48 8b 74 24 20       	mov    0x20(%rsp),%rsi
   cd835:	e8 06 3f fc ff       	call   91740 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cd83a:	48 89 c5             	mov    %rax,%rbp
   cd83d:	f0 49 ff 0e          	lock decq (%r14)
   cd841:	75 0b                	jne    cd84e <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x75e>
   cd843:	48 8d 7c 24 38       	lea    0x38(%rsp),%rdi
   cd848:	ff 15 52 1a 0a 00    	call   *0xa1a52(%rip)        # 16f2a0 <_DYNAMIC+0x2a0>
   cd84e:	48 8d bc 24 a8 00 00 	lea    0xa8(%rsp),%rdi
   cd855:	00 
   cd856:	48 89 de             	mov    %rbx,%rsi
   cd859:	4c 89 e2             	mov    %r12,%rdx
   cd85c:	48 89 e9             	mov    %rbp,%rcx
   cd85f:	e8 8c ec ff ff       	call   cc4f0 <<hyperreal::computable::node::RadicalCoefficient>::insert_atom>
   cd864:	49 83 c5 18          	add    $0x18,%r13
   cd868:	49 83 c7 e8          	add    $0xffffffffffffffe8,%r15
   cd86c:	75 92                	jne    cd800 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x710>
   cd86e:	31 db                	xor    %ebx,%ebx
   cd870:	48 8d 7c 24 70       	lea    0x70(%rsp),%rdi
   cd875:	e8 66 ff 03 00       	call   10d7e0 <<alloc::vec::into_iter::IntoIter<(hyperreal::computable::node::Computable, hyperreal::rational::arithmetic::Rational)> as core::ops::drop::Drop>::drop>
   cd87a:	80 7c 24 2c 00       	cmpb   $0x0,0x2c(%rsp)
   cd87f:	4c 8b 6c 24 40       	mov    0x40(%rsp),%r13
   cd884:	75 72                	jne    cd8f8 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x808>
   cd886:	48 8b 84 24 f0 00 00 	mov    0xf0(%rsp),%rax
   cd88d:	00 
   cd88e:	48 8b 58 20          	mov    0x20(%rax),%rbx
   cd892:	48 01 5c 24 30       	add    %rbx,0x30(%rsp)
   cd897:	4c 8d b4 24 a8 00 00 	lea    0xa8(%rsp),%r14
   cd89e:	00 
   cd89f:	90                   	nop
   cd8a0:	4c 8b 3b             	mov    (%rbx),%r15
   cd8a3:	4c 8b 63 08          	mov    0x8(%rbx),%r12
   cd8a7:	f0 49 ff 07          	lock incq (%r15)
   cd8ab:	0f 8e ff 03 00 00    	jle    cdcb0 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xbc0>
   cd8b1:	4d 85 e4             	test   %r12,%r12
   cd8b4:	74 0b                	je     cd8c1 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x7d1>
   cd8b6:	f0 49 ff 04 24       	lock incq (%r12)
   cd8bb:	0f 8e ef 03 00 00    	jle    cdcb0 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xbc0>
   cd8c1:	48 83 c3 10          	add    $0x10,%rbx
   cd8c5:	4c 89 7c 24 70       	mov    %r15,0x70(%rsp)
   cd8ca:	4c 89 64 24 78       	mov    %r12,0x78(%rsp)
   cd8cf:	48 89 df             	mov    %rbx,%rdi
   cd8d2:	48 8b 74 24 48       	mov    0x48(%rsp),%rsi
   cd8d7:	e8 64 3e fc ff       	call   91740 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cd8dc:	4c 89 f7             	mov    %r14,%rdi
   cd8df:	4c 89 fe             	mov    %r15,%rsi
   cd8e2:	4c 89 e2             	mov    %r12,%rdx
   cd8e5:	48 89 c1             	mov    %rax,%rcx
   cd8e8:	e8 03 ec ff ff       	call   cc4f0 <<hyperreal::computable::node::RadicalCoefficient>::insert_atom>
   cd8ed:	48 83 c3 08          	add    $0x8,%rbx
   cd8f1:	48 3b 5c 24 30       	cmp    0x30(%rsp),%rbx
   cd8f6:	75 a8                	jne    cd8a0 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x7b0>
   cd8f8:	48 8b 84 24 d8 00 00 	mov    0xd8(%rsp),%rax
   cd8ff:	00 
   cd900:	49 89 45 30          	mov    %rax,0x30(%r13)
   cd904:	0f 10 84 24 a8 00 00 	movups 0xa8(%rsp),%xmm0
   cd90b:	00 
   cd90c:	0f 10 8c 24 b8 00 00 	movups 0xb8(%rsp),%xmm1
   cd913:	00 
   cd914:	0f 10 94 24 c8 00 00 	movups 0xc8(%rsp),%xmm2
   cd91b:	00 
   cd91c:	41 0f 11 55 20       	movups %xmm2,0x20(%r13)
   cd921:	41 0f 11 4d 10       	movups %xmm1,0x10(%r13)
   cd926:	41 0f 11 45 00       	movups %xmm0,0x0(%r13)
   cd92b:	48 8b 7c 24 48       	mov    0x48(%rsp),%rdi
   cd930:	48 8b 07             	mov    (%rdi),%rax
   cd933:	f0 48 ff 08          	lock decq (%rax)
   cd937:	0f 85 1e fc ff ff    	jne    cd55b <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x46b>
   cd93d:	ff 15 5d 19 0a 00    	call   *0xa195d(%rip)        # 16f2a0 <_DYNAMIC+0x2a0>
   cd943:	e9 13 fc ff ff       	jmp    cd55b <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x46b>
   cd948:	48 89 de             	mov    %rbx,%rsi
   cd94b:	eb 18                	jmp    cd965 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x875>
   cd94d:	0f 1f 00             	nopl   (%rax)
   cd950:	49 89 f7             	mov    %rsi,%r15
   cd953:	4c 89 fe             	mov    %r15,%rsi
   cd956:	4d 8b 3f             	mov    (%r15),%r15
   cd959:	41 8b 47 60          	mov    0x60(%r15),%eax
   cd95d:	84 c0                	test   %al,%al
   cd95f:	0f 89 a2 f8 ff ff    	jns    cd207 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x117>
   cd965:	41 8b 47 60          	mov    0x60(%r15),%eax
   cd969:	84 c0                	test   %al,%al
   cd96b:	79 e3                	jns    cd950 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x860>
   cd96d:	41 8b 47 50          	mov    0x50(%r15),%eax
   cd971:	49 83 c7 40          	add    $0x40,%r15
   cd975:	85 c0                	test   %eax,%eax
   cd977:	74 da                	je     cd953 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x863>
   cd979:	4c 89 ff             	mov    %r15,%rdi
   cd97c:	e8 18 04 04 00       	call   10dd99 <<std::sync::once_lock::OnceLock<hyperreal::rational::arithmetic::CachedRationalProduct>>::initialize::<<std::sync::once_lock::OnceLock<hyperreal::rational::arithmetic::CachedRationalProduct>>::get_or_init<<hyperreal::rational::arithmetic::Rational>::canonicalized_ref::{closure#0}>::{closure#0}, !>>
   cd981:	eb d0                	jmp    cd953 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x863>
   cd983:	48 89 44 24 20       	mov    %rax,0x20(%rsp)
   cd988:	80 7e 64 01          	cmpb   $0x1,0x64(%rsi)
   cd98c:	75 08                	jne    cd996 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x8a6>
   cd98e:	b0 01                	mov    $0x1,%al
   cd990:	89 44 24 10          	mov    %eax,0x10(%rsp)
   cd994:	eb 26                	jmp    cd9bc <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x8cc>
   cd996:	c7 44 24 10 00 00 00 	movl   $0x0,0x10(%rsp)
   cd99d:	00 
   cd99e:	31 db                	xor    %ebx,%ebx
   cd9a0:	48 8b 7c 24 20       	mov    0x20(%rsp),%rdi
   cd9a5:	e8 66 24 fc ff       	call   8fe10 <<hyperreal::rational::arithmetic::Rational as core::ops::arith::Add>::add>
   cd9aa:	48 89 44 24 20       	mov    %rax,0x20(%rsp)
   cd9af:	48 89 44 24 50       	mov    %rax,0x50(%rsp)
   cd9b4:	c7 44 24 10 00 00 00 	movl   $0x0,0x10(%rsp)
   cd9bb:	00 
   cd9bc:	b3 01                	mov    $0x1,%bl
   cd9be:	48 8b bc 24 90 00 00 	mov    0x90(%rsp),%rdi
   cd9c5:	00 
   cd9c6:	4c 89 f6             	mov    %r14,%rsi
   cd9c9:	e8 72 3d fc ff       	call   91740 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cd9ce:	49 89 c6             	mov    %rax,%r14
   cd9d1:	48 8b 44 24 48       	mov    0x48(%rsp),%rax
   cd9d6:	48 89 44 24 70       	mov    %rax,0x70(%rsp)
   cd9db:	4c 89 b4 24 a8 00 00 	mov    %r14,0xa8(%rsp)
   cd9e2:	00 
   cd9e3:	48 8d bc 24 a8 00 00 	lea    0xa8(%rsp),%rdi
   cd9ea:	00 
   cd9eb:	4c 89 fe             	mov    %r15,%rsi
   cd9ee:	e8 4d 3d fc ff       	call   91740 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cd9f3:	48 89 c3             	mov    %rax,%rbx
   cd9f6:	f0 49 ff 0e          	lock decq (%r14)
   cd9fa:	4c 8b 74 24 60       	mov    0x60(%rsp),%r14
   cd9ff:	75 0e                	jne    cda0f <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x91f>
   cda01:	48 8d bc 24 a8 00 00 	lea    0xa8(%rsp),%rdi
   cda08:	00 
   cda09:	ff 15 91 18 0a 00    	call   *0xa1891(%rip)        # 16f2a0 <_DYNAMIC+0x2a0>
   cda0f:	48 8b 7c 24 48       	mov    0x48(%rsp),%rdi
   cda14:	48 89 de             	mov    %rbx,%rsi
   cda17:	e8 f4 23 fc ff       	call   8fe10 <<hyperreal::rational::arithmetic::Rational as core::ops::arith::Add>::add>
   cda1c:	48 89 c3             	mov    %rax,%rbx
   cda1f:	48 89 84 24 98 00 00 	mov    %rax,0x98(%rsp)
   cda26:	00 
   cda27:	48 8b 44 24 20       	mov    0x20(%rsp),%rax
   cda2c:	80 78 64 01          	cmpb   $0x1,0x64(%rax)
   cda30:	0f 85 87 00 00 00    	jne    cdabd <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x9cd>
   cda36:	48 8b 44 24 40       	mov    0x40(%rsp),%rax
   cda3b:	48 c7 00 00 00 00 00 	movq   $0x0,(%rax)
   cda42:	48 c7 40 08 08 00 00 	movq   $0x8,0x8(%rax)
   cda49:	00 
   cda4a:	0f 57 c0             	xorps  %xmm0,%xmm0
   cda4d:	0f 11 40 10          	movups %xmm0,0x10(%rax)
   cda51:	48 c7 40 20 08 00 00 	movq   $0x8,0x20(%rax)
   cda58:	00 
   cda59:	48 c7 40 28 00 00 00 	movq   $0x0,0x28(%rax)
   cda60:	00 
   cda61:	48 89 58 30          	mov    %rbx,0x30(%rax)
   cda65:	80 7c 24 10 00       	cmpb   $0x0,0x10(%rsp)
   cda6a:	4c 8b 64 24 20       	mov    0x20(%rsp),%r12
   cda6f:	48 8b 44 24 18       	mov    0x18(%rsp),%rax
   cda74:	74 1a                	je     cda90 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x9a0>
   cda76:	f0 48 ff 08          	lock decq (%rax)
   cda7a:	75 14                	jne    cda90 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x9a0>
   cda7c:	b0 01                	mov    $0x1,%al
   cda7e:	89 44 24 10          	mov    %eax,0x10(%rsp)
   cda82:	45 31 ff             	xor    %r15d,%r15d
   cda85:	48 8d 7c 24 38       	lea    0x38(%rsp),%rdi
   cda8a:	ff 15 10 18 0a 00    	call   *0xa1810(%rip)        # 16f2a0 <_DYNAMIC+0x2a0>
   cda90:	f0 49 ff 0c 24       	lock decq (%r12)
   cda95:	75 0b                	jne    cdaa2 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x9b2>
   cda97:	48 8d 7c 24 50       	lea    0x50(%rsp),%rdi
   cda9c:	ff 15 fe 17 0a 00    	call   *0xa17fe(%rip)        # 16f2a0 <_DYNAMIC+0x2a0>
   cdaa2:	41 b6 01             	mov    $0x1,%r14b
   cdaa5:	48 8b 5c 24 08       	mov    0x8(%rsp),%rbx
   cdaaa:	48 8b 43 30          	mov    0x30(%rbx),%rax
   cdaae:	f0 48 ff 08          	lock decq (%rax)
   cdab2:	0f 84 18 fa ff ff    	je     cd4d0 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x3e0>
   cdab8:	e9 1d fa ff ff       	jmp    cd4da <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x3ea>
   cdabd:	41 80 fc 01          	cmp    $0x1,%r12b
   cdac1:	75 3c                	jne    cdaff <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xa0f>
   cdac3:	49 8b 06             	mov    (%r14),%rax
   cdac6:	f0 48 ff 00          	lock incq (%rax)
   cdaca:	0f 8e e0 01 00 00    	jle    cdcb0 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xbc0>
   cdad0:	4d 8b 26             	mov    (%r14),%r12
   cdad3:	4c 89 a4 24 a8 00 00 	mov    %r12,0xa8(%rsp)
   cdada:	00 
   cdadb:	48 83 7d 00 00       	cmpq   $0x0,0x0(%rbp)
   cdae0:	0f 84 b9 01 00 00    	je     cdc9f <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xbaf>
   cdae6:	4d 8b 75 00          	mov    0x0(%r13),%r14
   cdaea:	49 8b 06             	mov    (%r14),%rax
   cdaed:	f0 48 ff 08          	lock decq (%rax)
   cdaf1:	75 09                	jne    cdafc <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xa0c>
   cdaf3:	4c 89 f7             	mov    %r14,%rdi
   cdaf6:	ff 15 a4 17 0a 00    	call   *0xa17a4(%rip)        # 16f2a0 <_DYNAMIC+0x2a0>
   cdafc:	4d 89 26             	mov    %r12,(%r14)
   cdaff:	4c 8b 74 24 30       	mov    0x30(%rsp),%r14
   cdb04:	49 8b 06             	mov    (%r14),%rax
   cdb07:	f0 48 ff 08          	lock decq (%rax)
   cdb0b:	4c 8b 64 24 20       	mov    0x20(%rsp),%r12
   cdb10:	75 09                	jne    cdb1b <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xa2b>
   cdb12:	4c 89 f7             	mov    %r14,%rdi
   cdb15:	ff 15 85 17 0a 00    	call   *0xa1785(%rip)        # 16f2a0 <_DYNAMIC+0x2a0>
   cdb1b:	49 89 1e             	mov    %rbx,(%r14)
   cdb1e:	4c 89 a4 24 a8 00 00 	mov    %r12,0xa8(%rsp)
   cdb25:	00 
   cdb26:	48 83 7d 00 00       	cmpq   $0x0,0x0(%rbp)
   cdb2b:	0f 84 5b 01 00 00    	je     cdc8c <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xb9c>
   cdb31:	49 8b 5d 00          	mov    0x0(%r13),%rbx
   cdb35:	48 8b 43 08          	mov    0x8(%rbx),%rax
   cdb39:	48 83 c3 08          	add    $0x8,%rbx
   cdb3d:	f0 48 ff 08          	lock decq (%rax)
   cdb41:	4c 8b 74 24 40       	mov    0x40(%rsp),%r14
   cdb46:	75 09                	jne    cdb51 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xa61>
   cdb48:	48 89 df             	mov    %rbx,%rdi
   cdb4b:	ff 15 4f 17 0a 00    	call   *0xa174f(%rip)        # 16f2a0 <_DYNAMIC+0x2a0>
   cdb51:	4c 89 23             	mov    %r12,(%rbx)
   cdb54:	48 8b 5c 24 08       	mov    0x8(%rsp),%rbx
   cdb59:	48 8d 7b 18          	lea    0x18(%rbx),%rdi
   cdb5d:	e8 de 1d fe ff       	call   af940 <core::ptr::drop_glue::<alloc::vec::Vec<(hyperreal::computable::node::Computable, hyperreal::rational::arithmetic::Rational)>>>
   cdb62:	48 c7 43 18 00 00 00 	movq   $0x0,0x18(%rbx)
   cdb69:	00 
   cdb6a:	48 c7 43 20 08 00 00 	movq   $0x8,0x20(%rbx)
   cdb71:	00 
   cdb72:	48 c7 43 28 00 00 00 	movq   $0x0,0x28(%rbx)
   cdb79:	00 
   cdb7a:	48 8b 43 30          	mov    0x30(%rbx),%rax
   cdb7e:	49 89 46 30          	mov    %rax,0x30(%r14)
   cdb82:	0f 10 03             	movups (%rbx),%xmm0
   cdb85:	41 0f 11 06          	movups %xmm0,(%r14)
   cdb89:	48 8b 43 20          	mov    0x20(%rbx),%rax
   cdb8d:	49 89 46 20          	mov    %rax,0x20(%r14)
   cdb91:	48 8b 43 28          	mov    0x28(%rbx),%rax
   cdb95:	49 89 46 28          	mov    %rax,0x28(%r14)
   cdb99:	48 8b 43 10          	mov    0x10(%rbx),%rax
   cdb9d:	49 89 46 10          	mov    %rax,0x10(%r14)
   cdba1:	48 8b 43 18          	mov    0x18(%rbx),%rax
   cdba5:	49 89 46 18          	mov    %rax,0x18(%r14)
   cdba9:	80 7c 24 10 00       	cmpb   $0x0,0x10(%rsp)
   cdbae:	48 8b 44 24 18       	mov    0x18(%rsp),%rax
   cdbb3:	0f 84 a2 f9 ff ff    	je     cd55b <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x46b>
   cdbb9:	f0 48 ff 08          	lock decq (%rax)
   cdbbd:	0f 85 98 f9 ff ff    	jne    cd55b <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x46b>
   cdbc3:	c7 44 24 10 00 00 00 	movl   $0x0,0x10(%rsp)
   cdbca:	00 
   cdbcb:	48 8d 7c 24 38       	lea    0x38(%rsp),%rdi
   cdbd0:	45 31 ff             	xor    %r15d,%r15d
   cdbd3:	ff 15 c7 16 0a 00    	call   *0xa16c7(%rip)        # 16f2a0 <_DYNAMIC+0x2a0>
   cdbd9:	e9 7d f9 ff ff       	jmp    cd55b <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x46b>
   cdbde:	4c 89 74 24 70       	mov    %r14,0x70(%rsp)
   cdbe3:	48 8d 44 24 70       	lea    0x70(%rsp),%rax
   cdbe8:	48 89 84 24 a8 00 00 	mov    %rax,0xa8(%rsp)
   cdbef:	00 
   cdbf0:	4c 89 f0             	mov    %r14,%rax
   cdbf3:	41 b6 01             	mov    $0x1,%r14b
   cdbf6:	48 8d 78 18          	lea    0x18(%rax),%rdi
   cdbfa:	48 8d 0d 8f d3 09 00 	lea    0x9d38f(%rip),%rcx        # 16af90 <anon.afeede234644f26776946e76b404b106.3.llvm.7695097067950034038+0x28>
   cdc01:	4c 8d 05 48 d3 09 00 	lea    0x9d348(%rip),%r8        # 16af50 <anon.afeede234644f26776946e76b404b106.2.llvm.7695097067950034038>
   cdc08:	48 8d 94 24 a8 00 00 	lea    0xa8(%rsp),%rdx
   cdc0f:	00 
   cdc10:	be 01 00 00 00       	mov    $0x1,%esi
   cdc15:	ff 15 25 16 0a 00    	call   *0xa1625(%rip)        # 16f240 <_DYNAMIC+0x240>
   cdc1b:	4c 8b 35 66 16 0a 00 	mov    0xa1666(%rip),%r14        # 16f288 <_DYNAMIC+0x288>
   cdc22:	49 8b 57 20          	mov    0x20(%r15),%rdx
   cdc26:	49 3b 56 10          	cmp    0x10(%r14),%rdx
   cdc2a:	0f 84 fb f5 ff ff    	je     cd22b <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x13b>
   cdc30:	e9 3c f6 ff ff       	jmp    cd271 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x181>
   cdc35:	4c 89 74 24 70       	mov    %r14,0x70(%rsp)
   cdc3a:	48 8d 44 24 70       	lea    0x70(%rsp),%rax
   cdc3f:	48 89 84 24 a8 00 00 	mov    %rax,0xa8(%rsp)
   cdc46:	00 
   cdc47:	4c 89 f0             	mov    %r14,%rax
   cdc4a:	41 b6 01             	mov    $0x1,%r14b
   cdc4d:	48 8d 78 18          	lea    0x18(%rax),%rdi
   cdc51:	48 8d 0d 38 d3 09 00 	lea    0x9d338(%rip),%rcx        # 16af90 <anon.afeede234644f26776946e76b404b106.3.llvm.7695097067950034038+0x28>
   cdc58:	4c 8d 05 f1 d2 09 00 	lea    0x9d2f1(%rip),%r8        # 16af50 <anon.afeede234644f26776946e76b404b106.2.llvm.7695097067950034038>
   cdc5f:	48 8d 94 24 a8 00 00 	lea    0xa8(%rsp),%rdx
   cdc66:	00 
   cdc67:	be 01 00 00 00       	mov    $0x1,%esi
   cdc6c:	ff 15 ce 15 0a 00    	call   *0xa15ce(%rip)        # 16f240 <_DYNAMIC+0x240>
   cdc72:	4c 8b 35 0f 16 0a 00 	mov    0xa160f(%rip),%r14        # 16f288 <_DYNAMIC+0x288>
   cdc79:	49 8b 57 38          	mov    0x38(%r15),%rdx
   cdc7d:	49 3b 56 10          	cmp    0x10(%r14),%rdx
   cdc81:	0f 84 d0 f5 ff ff    	je     cd257 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x167>
   cdc87:	e9 e5 f5 ff ff       	jmp    cd271 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x181>
   cdc8c:	48 8d 15 e5 d9 09 00 	lea    0x9d9e5(%rip),%rdx        # 16b678 <anon.afeede234644f26776946e76b404b106.261.llvm.7695097067950034038+0x140>
   cdc93:	31 ff                	xor    %edi,%edi
   cdc95:	31 f6                	xor    %esi,%esi
   cdc97:	ff 15 e3 1a 0a 00    	call   *0xa1ae3(%rip)        # 16f780 <_DYNAMIC+0x780>
   cdc9d:	eb 11                	jmp    cdcb0 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xbc0>
   cdc9f:	48 8d 15 ba d9 09 00 	lea    0x9d9ba(%rip),%rdx        # 16b660 <anon.afeede234644f26776946e76b404b106.261.llvm.7695097067950034038+0x128>
   cdca6:	31 ff                	xor    %edi,%edi
   cdca8:	31 f6                	xor    %esi,%esi
   cdcaa:	ff 15 d0 1a 0a 00    	call   *0xa1ad0(%rip)        # 16f780 <_DYNAMIC+0x780>
   cdcb0:	0f 0b                	ud2
   cdcb2:	eb 55                	jmp    cdd09 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xc19>
   cdcb4:	49 89 c7             	mov    %rax,%r15
   cdcb7:	4d 89 26             	mov    %r12,(%r14)
   cdcba:	eb 2e                	jmp    cdcea <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xbfa>
   cdcbc:	49 89 c7             	mov    %rax,%r15
   cdcbf:	4c 89 23             	mov    %r12,(%rbx)
   cdcc2:	e9 82 00 00 00       	jmp    cdd49 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xc59>
   cdcc7:	49 89 c7             	mov    %rax,%r15
   cdcca:	49 89 1e             	mov    %rbx,(%r14)
   cdccd:	e9 c2 00 00 00       	jmp    cdd94 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xca4>
   cdcd2:	49 89 c7             	mov    %rax,%r15
   cdcd5:	f0 49 ff 0c 24       	lock decq (%r12)
   cdcda:	75 0e                	jne    cdcea <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xbfa>
   cdcdc:	48 8d bc 24 a8 00 00 	lea    0xa8(%rsp),%rdi
   cdce3:	00 
   cdce4:	ff 15 b6 15 0a 00    	call   *0xa15b6(%rip)        # 16f2a0 <_DYNAMIC+0x2a0>
   cdcea:	f0 48 ff 0b          	lock decq (%rbx)
   cdcee:	b3 01                	mov    $0x1,%bl
   cdcf0:	0f 85 d4 00 00 00    	jne    cddca <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xcda>
   cdcf6:	48 8d bc 24 98 00 00 	lea    0x98(%rsp),%rdi
   cdcfd:	00 
   cdcfe:	ff 15 9c 15 0a 00    	call   *0xa159c(%rip)        # 16f2a0 <_DYNAMIC+0x2a0>
   cdd04:	e9 c1 00 00 00       	jmp    cddca <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xcda>
   cdd09:	49 89 c7             	mov    %rax,%r15
   cdd0c:	b3 01                	mov    $0x1,%bl
   cdd0e:	41 b6 01             	mov    $0x1,%r14b
   cdd11:	e9 7f 03 00 00       	jmp    ce095 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xfa5>
   cdd16:	49 89 c7             	mov    %rax,%r15
   cdd19:	48 8b 44 24 18       	mov    0x18(%rsp),%rax
   cdd1e:	48 89 44 24 50       	mov    %rax,0x50(%rsp)
   cdd23:	41 b5 01             	mov    $0x1,%r13b
   cdd26:	40 b5 01             	mov    $0x1,%bpl
   cdd29:	e9 f8 00 00 00       	jmp    cde26 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xd36>
   cdd2e:	49 89 c7             	mov    %rax,%r15
   cdd31:	48 c7 43 18 00 00 00 	movq   $0x0,0x18(%rbx)
   cdd38:	00 
   cdd39:	48 c7 43 20 08 00 00 	movq   $0x8,0x20(%rbx)
   cdd40:	00 
   cdd41:	48 c7 43 28 00 00 00 	movq   $0x0,0x28(%rbx)
   cdd48:	00 
   cdd49:	31 db                	xor    %ebx,%ebx
   cdd4b:	45 31 ed             	xor    %r13d,%r13d
   cdd4e:	e9 a5 00 00 00       	jmp    cddf8 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xd08>
   cdd53:	49 89 c7             	mov    %rax,%r15
   cdd56:	45 31 ed             	xor    %r13d,%r13d
   cdd59:	48 8b 44 24 20       	mov    0x20(%rsp),%rax
   cdd5e:	f0 48 ff 08          	lock decq (%rax)
   cdd62:	bb 00 00 00 00       	mov    $0x0,%ebx
   cdd67:	0f 85 8b 00 00 00    	jne    cddf8 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xd08>
   cdd6d:	31 db                	xor    %ebx,%ebx
   cdd6f:	48 8d bc 24 a8 00 00 	lea    0xa8(%rsp),%rdi
   cdd76:	00 
   cdd77:	ff 15 23 15 0a 00    	call   *0xa1523(%rip)        # 16f2a0 <_DYNAMIC+0x2a0>
   cdd7d:	45 31 ed             	xor    %r13d,%r13d
   cdd80:	eb 76                	jmp    cddf8 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xd08>
   cdd82:	49 89 c7             	mov    %rax,%r15
   cdd85:	eb 2b                	jmp    cddb2 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xcc2>
   cdd87:	e9 de 00 00 00       	jmp    cde6a <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xd7a>
   cdd8c:	e9 e4 00 00 00       	jmp    cde75 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xd85>
   cdd91:	49 89 c7             	mov    %rax,%r15
   cdd94:	b3 01                	mov    $0x1,%bl
   cdd96:	45 31 ed             	xor    %r13d,%r13d
   cdd99:	eb 5d                	jmp    cddf8 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xd08>
   cdd9b:	49 89 c7             	mov    %rax,%r15
   cdd9e:	f0 49 ff 0e          	lock decq (%r14)
   cdda2:	75 0e                	jne    cddb2 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xcc2>
   cdda4:	48 8d bc 24 a8 00 00 	lea    0xa8(%rsp),%rdi
   cddab:	00 
   cddac:	ff 15 ee 14 0a 00    	call   *0xa14ee(%rip)        # 16f2a0 <_DYNAMIC+0x2a0>
   cddb2:	48 8b 44 24 48       	mov    0x48(%rsp),%rax
   cddb7:	f0 48 ff 08          	lock decq (%rax)
   cddbb:	b3 01                	mov    $0x1,%bl
   cddbd:	75 0b                	jne    cddca <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xcda>
   cddbf:	48 8d 7c 24 70       	lea    0x70(%rsp),%rdi
   cddc4:	ff 15 d6 14 0a 00    	call   *0xa14d6(%rip)        # 16f2a0 <_DYNAMIC+0x2a0>
   cddca:	45 31 ed             	xor    %r13d,%r13d
   cddcd:	eb 29                	jmp    cddf8 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xd08>
   cddcf:	ff 15 d3 14 0a 00    	call   *0xa14d3(%rip)        # 16f2a8 <_DYNAMIC+0x2a8>
   cddd5:	49 89 c7             	mov    %rax,%r15
   cddd8:	40 b5 01             	mov    $0x1,%bpl
   cdddb:	eb 64                	jmp    cde41 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xd51>
   cdddd:	45 89 fd             	mov    %r15d,%r13d
   cdde0:	4c 89 64 24 20       	mov    %r12,0x20(%rsp)
   cdde5:	49 89 c7             	mov    %rax,%r15
   cdde8:	8b 44 24 10          	mov    0x10(%rsp),%eax
   cddec:	89 c3                	mov    %eax,%ebx
   cddee:	89 c5                	mov    %eax,%ebp
   cddf0:	eb 26                	jmp    cde18 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xd28>
   cddf2:	49 89 c7             	mov    %rax,%r15
   cddf5:	41 b5 01             	mov    $0x1,%r13b
   cddf8:	40 b5 01             	mov    $0x1,%bpl
   cddfb:	80 7c 24 10 00       	cmpb   $0x0,0x10(%rsp)
   cde00:	74 16                	je     cde18 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xd28>
   cde02:	48 8b 44 24 18       	mov    0x18(%rsp),%rax
   cde07:	f0 48 ff 08          	lock decq (%rax)
   cde0b:	75 0b                	jne    cde18 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xd28>
   cde0d:	48 8d 7c 24 38       	lea    0x38(%rsp),%rdi
   cde12:	ff 15 88 14 0a 00    	call   *0xa1488(%rip)        # 16f2a0 <_DYNAMIC+0x2a0>
   cde18:	84 db                	test   %bl,%bl
   cde1a:	74 20                	je     cde3c <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xd4c>
   cde1c:	48 8b 44 24 20       	mov    0x20(%rsp),%rax
   cde21:	48 89 44 24 18       	mov    %rax,0x18(%rsp)
   cde26:	48 8b 44 24 18       	mov    0x18(%rsp),%rax
   cde2b:	f0 48 ff 08          	lock decq (%rax)
   cde2f:	75 0b                	jne    cde3c <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xd4c>
   cde31:	48 8d 7c 24 50       	lea    0x50(%rsp),%rdi
   cde36:	ff 15 64 14 0a 00    	call   *0xa1464(%rip)        # 16f2a0 <_DYNAMIC+0x2a0>
   cde3c:	45 84 ed             	test   %r13b,%r13b
   cde3f:	74 16                	je     cde57 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xd67>
   cde41:	48 8b 44 24 48       	mov    0x48(%rsp),%rax
   cde46:	f0 48 ff 08          	lock decq (%rax)
   cde4a:	75 0b                	jne    cde57 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xd67>
   cde4c:	48 8d 7c 24 68       	lea    0x68(%rsp),%rdi
   cde51:	ff 15 49 14 0a 00    	call   *0xa1449(%rip)        # 16f2a0 <_DYNAMIC+0x2a0>
   cde57:	b3 01                	mov    $0x1,%bl
   cde59:	41 b6 01             	mov    $0x1,%r14b
   cde5c:	40 84 ed             	test   %bpl,%bpl
   cde5f:	0f 85 30 02 00 00    	jne    ce095 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xfa5>
   cde65:	e9 95 02 00 00       	jmp    ce0ff <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x100f>
   cde6a:	49 89 c7             	mov    %rax,%r15
   cde6d:	40 b5 01             	mov    $0x1,%bpl
   cde70:	e9 f2 01 00 00       	jmp    ce067 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xf77>
   cde75:	49 89 c7             	mov    %rax,%r15
   cde78:	40 b5 01             	mov    $0x1,%bpl
   cde7b:	e9 cb 01 00 00       	jmp    ce04b <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xf5b>
   cde80:	49 89 c7             	mov    %rax,%r15
   cde83:	45 31 f6             	xor    %r14d,%r14d
   cde86:	e9 74 01 00 00       	jmp    cdfff <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xf0f>
   cde8b:	49 89 c7             	mov    %rax,%r15
   cde8e:	45 84 f6             	test   %r14b,%r14b
   cde91:	75 52                	jne    cdee5 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xdf5>
   cde93:	e9 ac 00 00 00       	jmp    cdf44 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xe54>
   cde98:	49 89 c7             	mov    %rax,%r15
   cde9b:	eb 19                	jmp    cdeb6 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xdc6>
   cde9d:	49 89 c7             	mov    %rax,%r15
   cdea0:	eb 21                	jmp    cdec3 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xdd3>
   cdea2:	49 89 c7             	mov    %rax,%r15
   cdea5:	f0 49 ff 0e          	lock decq (%r14)
   cdea9:	75 0b                	jne    cdeb6 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xdc6>
   cdeab:	48 8d 7c 24 38       	lea    0x38(%rsp),%rdi
   cdeb0:	ff 15 ea 13 0a 00    	call   *0xa13ea(%rip)        # 16f2a0 <_DYNAMIC+0x2a0>
   cdeb6:	48 8d bc 24 98 00 00 	lea    0x98(%rsp),%rdi
   cdebd:	00 
   cdebe:	e8 dd 29 fe ff       	call   b08a0 <core::ptr::drop_glue::<hyperreal::computable::node::Computable>>
   cdec3:	31 db                	xor    %ebx,%ebx
   cdec5:	48 8d 7c 24 70       	lea    0x70(%rsp),%rdi
   cdeca:	e8 11 f9 03 00       	call   10d7e0 <<alloc::vec::into_iter::IntoIter<(hyperreal::computable::node::Computable, hyperreal::rational::arithmetic::Rational)> as core::ops::drop::Drop>::drop>
   cdecf:	45 31 f6             	xor    %r14d,%r14d
   cded2:	e9 28 01 00 00       	jmp    cdfff <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xf0f>
   cded7:	ff 15 cb 13 0a 00    	call   *0xa13cb(%rip)        # 16f2a8 <_DYNAMIC+0x2a8>
   cdedd:	4c 89 7c 24 58       	mov    %r15,0x58(%rsp)
   cdee2:	49 89 c7             	mov    %rax,%r15
   cdee5:	48 8b 7c 24 08       	mov    0x8(%rsp),%rdi
   cdeea:	e8 21 1b fe ff       	call   afa10 <core::ptr::drop_glue::<alloc::vec::Vec<(hyperreal::rational::arithmetic::Rational, hyperreal::rational::arithmetic::Rational)>>>
   cdeef:	eb 53                	jmp    cdf44 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xe54>
   cdef1:	49 89 c7             	mov    %rax,%r15
   cdef4:	48 8d 7c 24 70       	lea    0x70(%rsp),%rdi
   cdef9:	e8 a2 29 fe ff       	call   b08a0 <core::ptr::drop_glue::<hyperreal::computable::node::Computable>>
   cdefe:	eb 03                	jmp    cdf03 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xe13>
   cdf00:	49 89 c7             	mov    %rax,%r15
   cdf03:	31 db                	xor    %ebx,%ebx
   cdf05:	45 31 f6             	xor    %r14d,%r14d
   cdf08:	e9 f2 00 00 00       	jmp    cdfff <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xf0f>
   cdf0d:	49 89 c7             	mov    %rax,%r15
   cdf10:	b3 01                	mov    $0x1,%bl
   cdf12:	e9 7e 01 00 00       	jmp    ce095 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xfa5>
   cdf17:	49 89 c7             	mov    %rax,%r15
   cdf1a:	eb 77                	jmp    cdf93 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xea3>
   cdf1c:	4c 89 7c 24 58       	mov    %r15,0x58(%rsp)
   cdf21:	49 89 c7             	mov    %rax,%r15
   cdf24:	48 8b 33             	mov    (%rbx),%rsi
   cdf27:	48 85 f6             	test   %rsi,%rsi
   cdf2a:	74 18                	je     cdf44 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xe54>
   cdf2c:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
   cdf31:	48 8b 78 08          	mov    0x8(%rax),%rdi
   cdf35:	48 c1 e6 04          	shl    $0x4,%rsi
   cdf39:	ba 08 00 00 00       	mov    $0x8,%edx
   cdf3e:	ff 15 7c 13 0a 00    	call   *0xa137c(%rip)        # 16f2c0 <_DYNAMIC+0x2c0>
   cdf44:	48 8b 7c 24 58       	mov    0x58(%rsp),%rdi
   cdf49:	e8 f2 19 fe ff       	call   af940 <core::ptr::drop_glue::<alloc::vec::Vec<(hyperreal::computable::node::Computable, hyperreal::rational::arithmetic::Rational)>>>
   cdf4e:	e9 ac 01 00 00       	jmp    ce0ff <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x100f>
   cdf53:	49 89 c7             	mov    %rax,%r15
   cdf56:	f0 48 ff 0b          	lock decq (%rbx)
   cdf5a:	40 b5 01             	mov    $0x1,%bpl
   cdf5d:	0f 85 d2 00 00 00    	jne    ce035 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xf45>
   cdf63:	48 8d bc 24 98 00 00 	lea    0x98(%rsp),%rdi
   cdf6a:	00 
   cdf6b:	ff 15 2f 13 0a 00    	call   *0xa132f(%rip)        # 16f2a0 <_DYNAMIC+0x2a0>
   cdf71:	e9 bf 00 00 00       	jmp    ce035 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xf45>
   cdf76:	49 89 c7             	mov    %rax,%r15
   cdf79:	eb 29                	jmp    cdfa4 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xeb4>
   cdf7b:	49 89 c7             	mov    %rax,%r15
   cdf7e:	f0 49 ff 0c 24       	lock decq (%r12)
   cdf83:	75 0e                	jne    cdf93 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xea3>
   cdf85:	48 8d bc 24 98 00 00 	lea    0x98(%rsp),%rdi
   cdf8c:	00 
   cdf8d:	ff 15 0d 13 0a 00    	call   *0xa130d(%rip)        # 16f2a0 <_DYNAMIC+0x2a0>
   cdf93:	f0 48 ff 0b          	lock decq (%rbx)
   cdf97:	75 0b                	jne    cdfa4 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xeb4>
   cdf99:	48 8d 7c 24 38       	lea    0x38(%rsp),%rdi
   cdf9e:	ff 15 fc 12 0a 00    	call   *0xa12fc(%rip)        # 16f2a0 <_DYNAMIC+0x2a0>
   cdfa4:	31 ed                	xor    %ebp,%ebp
   cdfa6:	e9 8a 00 00 00       	jmp    ce035 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xf45>
   cdfab:	ff 15 f7 12 0a 00    	call   *0xa12f7(%rip)        # 16f2a8 <_DYNAMIC+0x2a8>
   cdfb1:	eb 1f                	jmp    cdfd2 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xee2>
   cdfb3:	49 89 c7             	mov    %rax,%r15
   cdfb6:	f0 49 ff 0e          	lock decq (%r14)
   cdfba:	b3 01                	mov    $0x1,%bl
   cdfbc:	75 3e                	jne    cdffc <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xf0c>
   cdfbe:	b3 01                	mov    $0x1,%bl
   cdfc0:	48 8d 7c 24 70       	lea    0x70(%rsp),%rdi
   cdfc5:	ff 15 d5 12 0a 00    	call   *0xa12d5(%rip)        # 16f2a0 <_DYNAMIC+0x2a0>
   cdfcb:	41 b6 01             	mov    $0x1,%r14b
   cdfce:	eb 2f                	jmp    cdfff <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xf0f>
   cdfd0:	eb 25                	jmp    cdff7 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xf07>
   cdfd2:	49 89 c7             	mov    %rax,%r15
   cdfd5:	40 b5 01             	mov    $0x1,%bpl
   cdfd8:	eb 5b                	jmp    ce035 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xf45>
   cdfda:	49 89 c7             	mov    %rax,%r15
   cdfdd:	f0 49 ff 0e          	lock decq (%r14)
   cdfe1:	b3 01                	mov    $0x1,%bl
   cdfe3:	75 17                	jne    cdffc <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xf0c>
   cdfe5:	b3 01                	mov    $0x1,%bl
   cdfe7:	48 8d 7c 24 70       	lea    0x70(%rsp),%rdi
   cdfec:	ff 15 ae 12 0a 00    	call   *0xa12ae(%rip)        # 16f2a0 <_DYNAMIC+0x2a0>
   cdff2:	41 b6 01             	mov    $0x1,%r14b
   cdff5:	eb 08                	jmp    cdfff <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xf0f>
   cdff7:	49 89 c7             	mov    %rax,%r15
   cdffa:	b3 01                	mov    $0x1,%bl
   cdffc:	41 b6 01             	mov    $0x1,%r14b
   cdfff:	48 8b 84 24 d8 00 00 	mov    0xd8(%rsp),%rax
   ce006:	00 
   ce007:	f0 48 ff 08          	lock decq (%rax)
   ce00b:	75 6e                	jne    ce07b <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xf8b>
   ce00d:	48 8d bc 24 d8 00 00 	lea    0xd8(%rsp),%rdi
   ce014:	00 
   ce015:	ff 15 85 12 0a 00    	call   *0xa1285(%rip)        # 16f2a0 <_DYNAMIC+0x2a0>
   ce01b:	eb 5e                	jmp    ce07b <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xf8b>
   ce01d:	49 89 c7             	mov    %rax,%r15
   ce020:	f0 49 ff 0c 24       	lock decq (%r12)
   ce025:	40 b5 01             	mov    $0x1,%bpl
   ce028:	75 0b                	jne    ce035 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xf45>
   ce02a:	48 8d 7c 24 50       	lea    0x50(%rsp),%rdi
   ce02f:	ff 15 6b 12 0a 00    	call   *0xa126b(%rip)        # 16f2a0 <_DYNAMIC+0x2a0>
   ce035:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
   ce03a:	f0 48 ff 08          	lock decq (%rax)
   ce03e:	75 0b                	jne    ce04b <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xf5b>
   ce040:	48 8d 7c 24 68       	lea    0x68(%rsp),%rdi
   ce045:	ff 15 55 12 0a 00    	call   *0xa1255(%rip)        # 16f2a0 <_DYNAMIC+0x2a0>
   ce04b:	48 8b 84 24 90 00 00 	mov    0x90(%rsp),%rax
   ce052:	00 
   ce053:	f0 48 ff 08          	lock decq (%rax)
   ce057:	75 0e                	jne    ce067 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xf77>
   ce059:	48 8d bc 24 e0 00 00 	lea    0xe0(%rsp),%rdi
   ce060:	00 
   ce061:	ff 15 39 12 0a 00    	call   *0xa1239(%rip)        # 16f2a0 <_DYNAMIC+0x2a0>
   ce067:	48 8d 7c 24 70       	lea    0x70(%rsp),%rdi
   ce06c:	e8 3f f8 03 00       	call   10d8b0 <<alloc::vec::into_iter::IntoIter<(hyperreal::rational::arithmetic::Rational, hyperreal::rational::arithmetic::Rational)> as core::ops::drop::Drop>::drop>
   ce071:	b3 01                	mov    $0x1,%bl
   ce073:	45 31 f6             	xor    %r14d,%r14d
   ce076:	40 84 ed             	test   %bpl,%bpl
   ce079:	75 84                	jne    cdfff <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xf0f>
   ce07b:	48 8d bc 24 a8 00 00 	lea    0xa8(%rsp),%rdi
   ce082:	00 
   ce083:	e8 88 19 fe ff       	call   afa10 <core::ptr::drop_glue::<alloc::vec::Vec<(hyperreal::rational::arithmetic::Rational, hyperreal::rational::arithmetic::Rational)>>>
   ce088:	48 8d bc 24 c0 00 00 	lea    0xc0(%rsp),%rdi
   ce08f:	00 
   ce090:	e8 ab 18 fe ff       	call   af940 <core::ptr::drop_glue::<alloc::vec::Vec<(hyperreal::computable::node::Computable, hyperreal::rational::arithmetic::Rational)>>>
   ce095:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
   ce09a:	48 8b 40 30          	mov    0x30(%rax),%rax
   ce09e:	f0 48 ff 08          	lock decq (%rax)
   ce0a2:	75 0f                	jne    ce0b3 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xfc3>
   ce0a4:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
   ce0a9:	48 8d 78 30          	lea    0x30(%rax),%rdi
   ce0ad:	ff 15 ed 11 0a 00    	call   *0xa11ed(%rip)        # 16f2a0 <_DYNAMIC+0x2a0>
   ce0b3:	45 84 f6             	test   %r14b,%r14b
   ce0b6:	74 0a                	je     ce0c2 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xfd2>
   ce0b8:	48 8b 7c 24 08       	mov    0x8(%rsp),%rdi
   ce0bd:	e8 4e 19 fe ff       	call   afa10 <core::ptr::drop_glue::<alloc::vec::Vec<(hyperreal::rational::arithmetic::Rational, hyperreal::rational::arithmetic::Rational)>>>
   ce0c2:	84 db                	test   %bl,%bl
   ce0c4:	74 39                	je     ce0ff <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x100f>
   ce0c6:	48 8b 7c 24 58       	mov    0x58(%rsp),%rdi
   ce0cb:	e8 70 18 fe ff       	call   af940 <core::ptr::drop_glue::<alloc::vec::Vec<(hyperreal::computable::node::Computable, hyperreal::rational::arithmetic::Rational)>>>
   ce0d0:	eb 2d                	jmp    ce0ff <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x100f>
   ce0d2:	ff 15 d0 11 0a 00    	call   *0xa11d0(%rip)        # 16f2a8 <_DYNAMIC+0x2a8>
   ce0d8:	4d 89 fd             	mov    %r15,%r13
   ce0db:	49 89 c7             	mov    %rax,%r15
   ce0de:	66 90                	xchg   %ax,%ax
   ce0e0:	49 ff ce             	dec    %r14
   ce0e3:	74 11                	je     ce0f6 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x1006>
   ce0e5:	4c 8d 63 18          	lea    0x18(%rbx),%r12
   ce0e9:	48 89 df             	mov    %rbx,%rdi
   ce0ec:	e8 bf 36 04 00       	call   1117b0 <core::ptr::drop_glue::<(hyperreal::computable::node::Computable, hyperreal::rational::arithmetic::Rational)>>
   ce0f1:	4c 89 e3             	mov    %r12,%rbx
   ce0f4:	eb ea                	jmp    ce0e0 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0xff0>
   ce0f6:	49 8b 45 00          	mov    0x0(%r13),%rax
   ce0fa:	48 85 c0             	test   %rax,%rax
   ce0fd:	75 08                	jne    ce107 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x1017>
   ce0ff:	4c 89 ff             	mov    %r15,%rdi
   ce102:	e8 69 ae 09 00       	call   168f70 <_Unwind_Resume@plt>
   ce107:	48 8b 4c 24 08       	mov    0x8(%rsp),%rcx
   ce10c:	48 8b 79 20          	mov    0x20(%rcx),%rdi
   ce110:	48 c1 e0 03          	shl    $0x3,%rax
   ce114:	48 8d 34 40          	lea    (%rax,%rax,2),%rsi
   ce118:	ba 08 00 00 00       	mov    $0x8,%edx
   ce11d:	ff 15 9d 11 0a 00    	call   *0xa119d(%rip)        # 16f2c0 <_DYNAMIC+0x2c0>
   ce123:	4c 89 ff             	mov    %r15,%rdi
   ce126:	e8 45 ae 09 00       	call   168f70 <_Unwind_Resume@plt>
   ce12b:	ff 15 77 11 0a 00    	call   *0xa1177(%rip)        # 16f2a8 <_DYNAMIC+0x2a8>
