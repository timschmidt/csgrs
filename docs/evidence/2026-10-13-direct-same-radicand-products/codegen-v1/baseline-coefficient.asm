
/home/tim/Documents/GitHub/workspace/csgrs/target/direct-same-radicand-products-v2/cost-v2-binaries/baseline-quadratic-time:     file format elf64-x86-64


Disassembly of section .text:

00000000000cd040 <<hyperreal::computable::node::RadicalCoefficient>::multiply>:
   cd040:	55                   	push   %rbp
   cd041:	41 57                	push   %r15
   cd043:	41 56                	push   %r14
   cd045:	41 55                	push   %r13
   cd047:	41 54                	push   %r12
   cd049:	53                   	push   %rbx
   cd04a:	48 81 ec f8 00 00 00 	sub    $0xf8,%rsp
   cd051:	48 89 fd             	mov    %rdi,%rbp
   cd054:	4c 8d 6e 18          	lea    0x18(%rsi),%r13
   cd058:	48 89 74 24 08       	mov    %rsi,0x8(%rsp)
   cd05d:	48 83 7e 28 00       	cmpq   $0x0,0x28(%rsi)
   cd062:	74 10                	je     cd074 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x34>
   cd064:	48 83 7a 10 00       	cmpq   $0x0,0x10(%rdx)
   cd069:	75 1e                	jne    cd089 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x49>
   cd06b:	48 83 7a 28 00       	cmpq   $0x0,0x28(%rdx)
   cd070:	75 17                	jne    cd089 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x49>
   cd072:	eb 3f                	jmp    cd0b3 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x73>
   cd074:	48 8b 4a 28          	mov    0x28(%rdx),%rcx
   cd078:	48 85 c9             	test   %rcx,%rcx
   cd07b:	74 36                	je     cd0b3 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x73>
   cd07d:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
   cd082:	48 83 78 10 00       	cmpq   $0x0,0x10(%rax)
   cd087:	74 3e                	je     cd0c7 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x87>
   cd089:	48 c7 45 00 ff ff ff 	movq   $0xffffffffffffffff,0x0(%rbp)
   cd090:	ff 
   cd091:	48 8b 5c 24 08       	mov    0x8(%rsp),%rbx
   cd096:	48 8b 43 30          	mov    0x30(%rbx),%rax
   cd09a:	f0 48 ff 08          	lock decq (%rax)
   cd09e:	0f 85 dd 05 00 00    	jne    cd681 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x641>
   cd0a4:	48 8d 7b 30          	lea    0x30(%rbx),%rdi
   cd0a8:	ff 15 72 1a 0a 00    	call   *0xa1a72(%rip)        # 16eb20 <_DYNAMIC+0x2a0>
   cd0ae:	e9 ce 05 00 00       	jmp    cd681 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x641>
   cd0b3:	b0 01                	mov    $0x1,%al
   cd0b5:	89 44 24 1c          	mov    %eax,0x1c(%rsp)
   cd0b9:	48 c7 84 24 80 00 00 	movq   $0x0,0x80(%rsp)
   cd0c0:	00 00 00 00 00 
   cd0c5:	eb 10                	jmp    cd0d7 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x97>
   cd0c7:	48 89 8c 24 80 00 00 	mov    %rcx,0x80(%rsp)
   cd0ce:	00 
   cd0cf:	c7 44 24 1c 00 00 00 	movl   $0x0,0x1c(%rsp)
   cd0d6:	00 
   cd0d7:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
   cd0dc:	48 8d 78 30          	lea    0x30(%rax),%rdi
   cd0e0:	48 89 94 24 c8 00 00 	mov    %rdx,0xc8(%rsp)
   cd0e7:	00 
   cd0e8:	48 8d 72 30          	lea    0x30(%rdx),%rsi
   cd0ec:	b0 01                	mov    $0x1,%al
   cd0ee:	89 44 24 04          	mov    %eax,0x4(%rsp)
   cd0f2:	b3 01                	mov    $0x1,%bl
   cd0f4:	48 89 7c 24 20       	mov    %rdi,0x20(%rsp)
   cd0f9:	48 89 b4 24 d0 00 00 	mov    %rsi,0xd0(%rsp)
   cd100:	00 
   cd101:	e8 8a 45 fc ff       	call   91690 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cd106:	48 89 84 24 b8 00 00 	mov    %rax,0xb8(%rsp)
   cd10d:	00 
   cd10e:	48 c7 84 24 88 00 00 	movq   $0x0,0x88(%rsp)
   cd115:	00 00 00 00 00 
   cd11a:	48 c7 84 24 90 00 00 	movq   $0x8,0x90(%rsp)
   cd121:	00 08 00 00 00 
   cd126:	0f 57 c0             	xorps  %xmm0,%xmm0
   cd129:	0f 11 84 24 98 00 00 	movups %xmm0,0x98(%rsp)
   cd130:	00 
   cd131:	48 c7 84 24 a8 00 00 	movq   $0x8,0xa8(%rsp)
   cd138:	00 08 00 00 00 
   cd13d:	48 c7 84 24 b0 00 00 	movq   $0x0,0xb0(%rsp)
   cd144:	00 00 00 00 00 
   cd149:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
   cd14e:	4c 8b 70 08          	mov    0x8(%rax),%r14
   cd152:	4c 8b 60 10          	mov    0x10(%rax),%r12
   cd156:	49 c1 e4 04          	shl    $0x4,%r12
   cd15a:	48 8d 9c 24 88 00 00 	lea    0x88(%rsp),%rbx
   cd161:	00 
   cd162:	66 66 66 66 66 2e 0f 	data16 data16 data16 data16 cs nopw 0x0(%rax,%rax,1)
   cd169:	1f 84 00 00 00 00 00 
   cd170:	4d 85 e4             	test   %r12,%r12
   cd173:	74 4f                	je     cd1c4 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x184>
   cd175:	49 8b 06             	mov    (%r14),%rax
   cd178:	f0 48 ff 00          	lock incq (%rax)
   cd17c:	0f 8e 7c 05 00 00    	jle    cd6fe <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x6be>
   cd182:	49 8d 7e 08          	lea    0x8(%r14),%rdi
   cd186:	4d 8b 3e             	mov    (%r14),%r15
   cd189:	4c 89 7c 24 40       	mov    %r15,0x40(%rsp)
   cd18e:	48 8b b4 24 d0 00 00 	mov    0xd0(%rsp),%rsi
   cd195:	00 
   cd196:	e8 f5 44 fc ff       	call   91690 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cd19b:	48 89 df             	mov    %rbx,%rdi
   cd19e:	4c 89 fe             	mov    %r15,%rsi
   cd1a1:	48 89 c2             	mov    %rax,%rdx
   cd1a4:	e8 e7 f5 ff ff       	call   cc790 <<hyperreal::computable::node::RadicalCoefficient>::insert_radical>
   cd1a9:	49 83 c6 10          	add    $0x10,%r14
   cd1ad:	49 83 c4 f0          	add    $0xfffffffffffffff0,%r12
   cd1b1:	84 c0                	test   %al,%al
   cd1b3:	75 bb                	jne    cd170 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x130>
   cd1b5:	48 c7 45 00 ff ff ff 	movq   $0xffffffffffffffff,0x0(%rbp)
   cd1bc:	ff 
   cd1bd:	b3 01                	mov    $0x1,%bl
   cd1bf:	e9 93 04 00 00       	jmp    cd657 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x617>
   cd1c4:	4c 89 6c 24 60       	mov    %r13,0x60(%rsp)
   cd1c9:	48 8b 84 24 c8 00 00 	mov    0xc8(%rsp),%rax
   cd1d0:	00 
   cd1d1:	4c 8b 68 08          	mov    0x8(%rax),%r13
   cd1d5:	4c 8b 60 10          	mov    0x10(%rax),%r12
   cd1d9:	4d 89 e6             	mov    %r12,%r14
   cd1dc:	49 c1 e6 04          	shl    $0x4,%r14
   cd1e0:	48 8d 9c 24 88 00 00 	lea    0x88(%rsp),%rbx
   cd1e7:	00 
   cd1e8:	4c 89 b4 24 e0 00 00 	mov    %r14,0xe0(%rsp)
   cd1ef:	00 
   cd1f0:	4c 89 ac 24 e8 00 00 	mov    %r13,0xe8(%rsp)
   cd1f7:	00 
   cd1f8:	0f 1f 84 00 00 00 00 	nopl   0x0(%rax,%rax,1)
   cd1ff:	00 
   cd200:	4d 85 f6             	test   %r14,%r14
   cd203:	74 4e                	je     cd253 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x213>
   cd205:	49 8b 45 00          	mov    0x0(%r13),%rax
   cd209:	f0 48 ff 00          	lock incq (%rax)
   cd20d:	0f 8e eb 04 00 00    	jle    cd6fe <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x6be>
   cd213:	49 8d 7d 08          	lea    0x8(%r13),%rdi
   cd217:	4d 8b 7d 00          	mov    0x0(%r13),%r15
   cd21b:	4c 89 7c 24 40       	mov    %r15,0x40(%rsp)
   cd220:	48 8b 74 24 20       	mov    0x20(%rsp),%rsi
   cd225:	e8 66 44 fc ff       	call   91690 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cd22a:	48 89 df             	mov    %rbx,%rdi
   cd22d:	4c 89 fe             	mov    %r15,%rsi
   cd230:	48 89 c2             	mov    %rax,%rdx
   cd233:	e8 58 f5 ff ff       	call   cc790 <<hyperreal::computable::node::RadicalCoefficient>::insert_radical>
   cd238:	49 83 c5 10          	add    $0x10,%r13
   cd23c:	49 83 c6 f0          	add    $0xfffffffffffffff0,%r14
   cd240:	84 c0                	test   %al,%al
   cd242:	75 bc                	jne    cd200 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x1c0>
   cd244:	48 c7 45 00 ff ff ff 	movq   $0xffffffffffffffff,0x0(%rbp)
   cd24b:	ff 
   cd24c:	b3 01                	mov    $0x1,%bl
   cd24e:	e9 fb 03 00 00       	jmp    cd64e <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x60e>
   cd253:	48 8b 4c 24 08       	mov    0x8(%rsp),%rcx
   cd258:	48 8b 01             	mov    (%rcx),%rax
   cd25b:	4c 8b 71 08          	mov    0x8(%rcx),%r14
   cd25f:	48 8b 49 10          	mov    0x10(%rcx),%rcx
   cd263:	49 89 cd             	mov    %rcx,%r13
   cd266:	49 c1 e5 04          	shl    $0x4,%r13
   cd26a:	4b 8d 14 2e          	lea    (%r14,%r13,1),%rdx
   cd26e:	4c 89 74 24 40       	mov    %r14,0x40(%rsp)
   cd273:	4c 89 74 24 48       	mov    %r14,0x48(%rsp)
   cd278:	48 89 44 24 50       	mov    %rax,0x50(%rsp)
   cd27d:	48 89 94 24 d8 00 00 	mov    %rdx,0xd8(%rsp)
   cd284:	00 
   cd285:	48 89 54 24 58       	mov    %rdx,0x58(%rsp)
   cd28a:	48 85 c9             	test   %rcx,%rcx
   cd28d:	48 89 ac 24 c0 00 00 	mov    %rbp,0xc0(%rsp)
   cd294:	00 
   cd295:	0f 84 c2 01 00 00    	je     cd45d <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x41d>
   cd29b:	4d 85 e4             	test   %r12,%r12
   cd29e:	75 77                	jne    cd317 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x2d7>
   cd2a0:	49 83 c6 10          	add    $0x10,%r14
   cd2a4:	48 8d 5c 24 28       	lea    0x28(%rsp),%rbx
   cd2a9:	4c 8b 25 70 18 0a 00 	mov    0xa1870(%rip),%r12        # 16eb20 <_DYNAMIC+0x2a0>
   cd2b0:	4c 8d 7c 24 10       	lea    0x10(%rsp),%r15
   cd2b5:	eb 17                	jmp    cd2ce <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x28e>
   cd2b7:	66 0f 1f 84 00 00 00 	nopw   0x0(%rax,%rax,1)
   cd2be:	00 00 
   cd2c0:	49 83 c6 10          	add    $0x10,%r14
   cd2c4:	49 83 c5 f0          	add    $0xfffffffffffffff0,%r13
   cd2c8:	0f 84 8f 01 00 00    	je     cd45d <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x41d>
   cd2ce:	4c 89 74 24 48       	mov    %r14,0x48(%rsp)
   cd2d3:	49 8b 4e f0          	mov    -0x10(%r14),%rcx
   cd2d7:	49 8b 46 f8          	mov    -0x8(%r14),%rax
   cd2db:	48 89 4c 24 30       	mov    %rcx,0x30(%rsp)
   cd2e0:	48 89 4c 24 10       	mov    %rcx,0x10(%rsp)
   cd2e5:	48 89 44 24 28       	mov    %rax,0x28(%rsp)
   cd2ea:	f0 48 ff 08          	lock decq (%rax)
   cd2ee:	75 06                	jne    cd2f6 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x2b6>
   cd2f0:	48 89 df             	mov    %rbx,%rdi
   cd2f3:	41 ff d4             	call   *%r12
   cd2f6:	48 8b 44 24 30       	mov    0x30(%rsp),%rax
   cd2fb:	f0 48 ff 08          	lock decq (%rax)
   cd2ff:	75 bf                	jne    cd2c0 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x280>
   cd301:	4c 89 ff             	mov    %r15,%rdi
   cd304:	41 ff d4             	call   *%r12
   cd307:	eb b7                	jmp    cd2c0 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x280>
   cd309:	4c 3b b4 24 d8 00 00 	cmp    0xd8(%rsp),%r14
   cd310:	00 
   cd311:	0f 84 46 01 00 00    	je     cd45d <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x41d>
   cd317:	4c 89 f0             	mov    %r14,%rax
   cd31a:	49 83 c6 10          	add    $0x10,%r14
   cd31e:	4c 89 74 24 48       	mov    %r14,0x48(%rsp)
   cd323:	48 8b 08             	mov    (%rax),%rcx
   cd326:	48 8b 40 08          	mov    0x8(%rax),%rax
   cd32a:	48 89 4c 24 30       	mov    %rcx,0x30(%rsp)
   cd32f:	48 89 4c 24 10       	mov    %rcx,0x10(%rsp)
   cd334:	48 89 44 24 78       	mov    %rax,0x78(%rsp)
   cd339:	48 89 44 24 28       	mov    %rax,0x28(%rsp)
   cd33e:	48 8b ac 24 e0 00 00 	mov    0xe0(%rsp),%rbp
   cd345:	00 
   cd346:	4c 8b bc 24 e8 00 00 	mov    0xe8(%rsp),%r15
   cd34d:	00 
   cd34e:	eb 0e                	jmp    cd35e <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x31e>
   cd350:	49 83 c7 10          	add    $0x10,%r15
   cd354:	48 83 c5 f0          	add    $0xfffffffffffffff0,%rbp
   cd358:	0f 84 ca 00 00 00    	je     cd428 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x3e8>
   cd35e:	49 8d 77 08          	lea    0x8(%r15),%rsi
   cd362:	48 8d 7c 24 28       	lea    0x28(%rsp),%rdi
   cd367:	e8 24 43 fc ff       	call   91690 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cd36c:	48 89 c3             	mov    %rax,%rbx
   cd36f:	48 89 84 24 f0 00 00 	mov    %rax,0xf0(%rsp)
   cd376:	00 
   cd377:	48 8d 7c 24 10       	lea    0x10(%rsp),%rdi
   cd37c:	4c 89 fe             	mov    %r15,%rsi
   cd37f:	ff 15 43 17 0a 00    	call   *0xa1743(%rip)        # 16eac8 <_DYNAMIC+0x248>
   cd385:	84 c0                	test   %al,%al
   cd387:	74 57                	je     cd3e0 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x3a0>
   cd389:	4c 8b a4 24 b8 00 00 	mov    0xb8(%rsp),%r12
   cd390:	00 
   cd391:	4c 89 64 24 38       	mov    %r12,0x38(%rsp)
   cd396:	48 89 5c 24 68       	mov    %rbx,0x68(%rsp)
   cd39b:	48 8d 7c 24 68       	lea    0x68(%rsp),%rdi
   cd3a0:	48 8d 74 24 10       	lea    0x10(%rsp),%rsi
   cd3a5:	e8 e6 42 fc ff       	call   91690 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cd3aa:	49 89 c5             	mov    %rax,%r13
   cd3ad:	f0 48 ff 0b          	lock decq (%rbx)
   cd3b1:	75 0b                	jne    cd3be <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x37e>
   cd3b3:	48 8d 7c 24 68       	lea    0x68(%rsp),%rdi
   cd3b8:	ff 15 62 17 0a 00    	call   *0xa1762(%rip)        # 16eb20 <_DYNAMIC+0x2a0>
   cd3be:	4c 89 e7             	mov    %r12,%rdi
   cd3c1:	4c 89 ee             	mov    %r13,%rsi
   cd3c4:	e8 97 29 fc ff       	call   8fd60 <<hyperreal::rational::arithmetic::Rational as core::ops::arith::Add>::add>
   cd3c9:	48 89 84 24 b8 00 00 	mov    %rax,0xb8(%rsp)
   cd3d0:	00 
   cd3d1:	e9 7a ff ff ff       	jmp    cd350 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x310>
   cd3d6:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
   cd3dd:	00 00 00 
   cd3e0:	48 8d 7c 24 10       	lea    0x10(%rsp),%rdi
   cd3e5:	4c 89 fe             	mov    %r15,%rsi
   cd3e8:	e8 a3 42 fc ff       	call   91690 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cd3ed:	48 89 c7             	mov    %rax,%rdi
   cd3f0:	e8 6b 5e fa ff       	call   73260 <<hyperreal::rational::arithmetic::Rational>::extract_square_reduced_retained>
   cd3f5:	49 89 d4             	mov    %rdx,%r12
   cd3f8:	48 89 54 24 68       	mov    %rdx,0x68(%rsp)
   cd3fd:	48 89 df             	mov    %rbx,%rdi
   cd400:	48 89 c6             	mov    %rax,%rsi
   cd403:	e8 08 4e fc ff       	call   92210 <<hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cd408:	48 8d bc 24 88 00 00 	lea    0x88(%rsp),%rdi
   cd40f:	00 
   cd410:	4c 89 e6             	mov    %r12,%rsi
   cd413:	48 89 c2             	mov    %rax,%rdx
   cd416:	e8 75 f3 ff ff       	call   cc790 <<hyperreal::computable::node::RadicalCoefficient>::insert_radical>
   cd41b:	84 c0                	test   %al,%al
   cd41d:	0f 85 2d ff ff ff    	jne    cd350 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x310>
   cd423:	e9 dc 01 00 00       	jmp    cd604 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x5c4>
   cd428:	48 8b 44 24 78       	mov    0x78(%rsp),%rax
   cd42d:	f0 48 ff 08          	lock decq (%rax)
   cd431:	75 0b                	jne    cd43e <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x3fe>
   cd433:	48 8d 7c 24 28       	lea    0x28(%rsp),%rdi
   cd438:	ff 15 e2 16 0a 00    	call   *0xa16e2(%rip)        # 16eb20 <_DYNAMIC+0x2a0>
   cd43e:	48 8b 44 24 30       	mov    0x30(%rsp),%rax
   cd443:	f0 48 ff 08          	lock decq (%rax)
   cd447:	0f 85 bc fe ff ff    	jne    cd309 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x2c9>
   cd44d:	48 8d 7c 24 10       	lea    0x10(%rsp),%rdi
   cd452:	ff 15 c8 16 0a 00    	call   *0xa16c8(%rip)        # 16eb20 <_DYNAMIC+0x2a0>
   cd458:	e9 ac fe ff ff       	jmp    cd309 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x2c9>
   cd45d:	41 b6 01             	mov    $0x1,%r14b
   cd460:	48 8d 7c 24 40       	lea    0x40(%rsp),%rdi
   cd465:	e8 f6 fc 03 00       	call   10d160 <<alloc::vec::into_iter::IntoIter<(hyperreal::rational::arithmetic::Rational, hyperreal::rational::arithmetic::Rational)> as core::ops::drop::Drop>::drop>
   cd46a:	48 8b 4c 24 08       	mov    0x8(%rsp),%rcx
   cd46f:	48 8b 41 18          	mov    0x18(%rcx),%rax
   cd473:	4c 8b 71 20          	mov    0x20(%rcx),%r14
   cd477:	48 8b 49 28          	mov    0x28(%rcx),%rcx
   cd47b:	48 8d 14 cd 00 00 00 	lea    0x0(,%rcx,8),%rdx
   cd482:	00 
   cd483:	4c 8d 2c 52          	lea    (%rdx,%rdx,2),%r13
   cd487:	4b 8d 14 2e          	lea    (%r14,%r13,1),%rdx
   cd48b:	4c 89 74 24 40       	mov    %r14,0x40(%rsp)
   cd490:	4c 89 74 24 48       	mov    %r14,0x48(%rsp)
   cd495:	48 89 44 24 50       	mov    %rax,0x50(%rsp)
   cd49a:	48 89 54 24 58       	mov    %rdx,0x58(%rsp)
   cd49f:	48 85 c9             	test   %rcx,%rcx
   cd4a2:	74 77                	je     cd51b <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x4db>
   cd4a4:	49 83 c6 18          	add    $0x18,%r14
   cd4a8:	0f 1f 84 00 00 00 00 	nopl   0x0(%rax,%rax,1)
   cd4af:	00 
   cd4b0:	4c 89 74 24 48       	mov    %r14,0x48(%rsp)
   cd4b5:	4d 8b 66 e8          	mov    -0x18(%r14),%r12
   cd4b9:	4d 85 e4             	test   %r12,%r12
   cd4bc:	74 5d                	je     cd51b <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x4db>
   cd4be:	49 8b 6e f0          	mov    -0x10(%r14),%rbp
   cd4c2:	49 8b 5e f8          	mov    -0x8(%r14),%rbx
   cd4c6:	4c 89 64 24 68       	mov    %r12,0x68(%rsp)
   cd4cb:	48 89 6c 24 70       	mov    %rbp,0x70(%rsp)
   cd4d0:	48 89 5c 24 38       	mov    %rbx,0x38(%rsp)
   cd4d5:	48 8d 7c 24 38       	lea    0x38(%rsp),%rdi
   cd4da:	48 8b b4 24 d0 00 00 	mov    0xd0(%rsp),%rsi
   cd4e1:	00 
   cd4e2:	e8 a9 41 fc ff       	call   91690 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cd4e7:	49 89 c7             	mov    %rax,%r15
   cd4ea:	f0 48 ff 0b          	lock decq (%rbx)
   cd4ee:	75 0b                	jne    cd4fb <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x4bb>
   cd4f0:	48 8d 7c 24 38       	lea    0x38(%rsp),%rdi
   cd4f5:	ff 15 25 16 0a 00    	call   *0xa1625(%rip)        # 16eb20 <_DYNAMIC+0x2a0>
   cd4fb:	48 8d bc 24 88 00 00 	lea    0x88(%rsp),%rdi
   cd502:	00 
   cd503:	4c 89 e6             	mov    %r12,%rsi
   cd506:	48 89 ea             	mov    %rbp,%rdx
   cd509:	4c 89 f9             	mov    %r15,%rcx
   cd50c:	e8 2f ef ff ff       	call   cc440 <<hyperreal::computable::node::RadicalCoefficient>::insert_atom>
   cd511:	49 83 c6 18          	add    $0x18,%r14
   cd515:	49 83 c5 e8          	add    $0xffffffffffffffe8,%r13
   cd519:	75 95                	jne    cd4b0 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x470>
   cd51b:	45 31 f6             	xor    %r14d,%r14d
   cd51e:	48 8d 7c 24 40       	lea    0x40(%rsp),%rdi
   cd523:	e8 68 fb 03 00       	call   10d090 <<alloc::vec::into_iter::IntoIter<(hyperreal::computable::node::Computable, hyperreal::rational::arithmetic::Rational)> as core::ops::drop::Drop>::drop>
   cd528:	80 7c 24 1c 00       	cmpb   $0x0,0x1c(%rsp)
   cd52d:	48 8b ac 24 c0 00 00 	mov    0xc0(%rsp),%rbp
   cd534:	00 
   cd535:	0f 85 7c 00 00 00    	jne    cd5b7 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x577>
   cd53b:	48 8b 84 24 c8 00 00 	mov    0xc8(%rsp),%rax
   cd542:	00 
   cd543:	48 8b 58 20          	mov    0x20(%rax),%rbx
   cd547:	48 8b 84 24 80 00 00 	mov    0x80(%rsp),%rax
   cd54e:	00 
   cd54f:	48 8d 04 40          	lea    (%rax,%rax,2),%rax
   cd553:	4c 8d 34 c3          	lea    (%rbx,%rax,8),%r14
   cd557:	4c 8d bc 24 88 00 00 	lea    0x88(%rsp),%r15
   cd55e:	00 
   cd55f:	90                   	nop
   cd560:	4c 8b 23             	mov    (%rbx),%r12
   cd563:	4c 8b 6b 08          	mov    0x8(%rbx),%r13
   cd567:	f0 49 ff 04 24       	lock incq (%r12)
   cd56c:	0f 8e 8c 01 00 00    	jle    cd6fe <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x6be>
   cd572:	4d 85 ed             	test   %r13,%r13
   cd575:	48 8b 74 24 20       	mov    0x20(%rsp),%rsi
   cd57a:	74 0b                	je     cd587 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x547>
   cd57c:	f0 49 ff 45 00       	lock incq 0x0(%r13)
   cd581:	0f 8e 77 01 00 00    	jle    cd6fe <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x6be>
   cd587:	48 83 c3 10          	add    $0x10,%rbx
   cd58b:	4c 89 64 24 40       	mov    %r12,0x40(%rsp)
   cd590:	4c 89 6c 24 48       	mov    %r13,0x48(%rsp)
   cd595:	48 89 df             	mov    %rbx,%rdi
   cd598:	e8 f3 40 fc ff       	call   91690 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cd59d:	4c 89 ff             	mov    %r15,%rdi
   cd5a0:	4c 89 e6             	mov    %r12,%rsi
   cd5a3:	4c 89 ea             	mov    %r13,%rdx
   cd5a6:	48 89 c1             	mov    %rax,%rcx
   cd5a9:	e8 92 ee ff ff       	call   cc440 <<hyperreal::computable::node::RadicalCoefficient>::insert_atom>
   cd5ae:	48 83 c3 08          	add    $0x8,%rbx
   cd5b2:	4c 39 f3             	cmp    %r14,%rbx
   cd5b5:	75 a9                	jne    cd560 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x520>
   cd5b7:	48 8b 84 24 b8 00 00 	mov    0xb8(%rsp),%rax
   cd5be:	00 
   cd5bf:	48 89 45 30          	mov    %rax,0x30(%rbp)
   cd5c3:	0f 10 84 24 88 00 00 	movups 0x88(%rsp),%xmm0
   cd5ca:	00 
   cd5cb:	0f 10 8c 24 98 00 00 	movups 0x98(%rsp),%xmm1
   cd5d2:	00 
   cd5d3:	0f 10 94 24 a8 00 00 	movups 0xa8(%rsp),%xmm2
   cd5da:	00 
   cd5db:	0f 11 55 20          	movups %xmm2,0x20(%rbp)
   cd5df:	0f 11 4d 10          	movups %xmm1,0x10(%rbp)
   cd5e3:	0f 11 45 00          	movups %xmm0,0x0(%rbp)
   cd5e7:	48 8b 7c 24 20       	mov    0x20(%rsp),%rdi
   cd5ec:	48 8b 07             	mov    (%rdi),%rax
   cd5ef:	f0 48 ff 08          	lock decq (%rax)
   cd5f3:	0f 85 f3 00 00 00    	jne    cd6ec <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x6ac>
   cd5f9:	ff 15 21 15 0a 00    	call   *0xa1521(%rip)        # 16eb20 <_DYNAMIC+0x2a0>
   cd5ff:	e9 e8 00 00 00       	jmp    cd6ec <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x6ac>
   cd604:	48 8b 84 24 c0 00 00 	mov    0xc0(%rsp),%rax
   cd60b:	00 
   cd60c:	48 c7 00 ff ff ff ff 	movq   $0xffffffffffffffff,(%rax)
   cd613:	48 8b 44 24 78       	mov    0x78(%rsp),%rax
   cd618:	f0 48 ff 08          	lock decq (%rax)
   cd61c:	75 0b                	jne    cd629 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x5e9>
   cd61e:	48 8d 7c 24 28       	lea    0x28(%rsp),%rdi
   cd623:	ff 15 f7 14 0a 00    	call   *0xa14f7(%rip)        # 16eb20 <_DYNAMIC+0x2a0>
   cd629:	48 8b 44 24 30       	mov    0x30(%rsp),%rax
   cd62e:	f0 48 ff 08          	lock decq (%rax)
   cd632:	75 0b                	jne    cd63f <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x5ff>
   cd634:	48 8d 7c 24 10       	lea    0x10(%rsp),%rdi
   cd639:	ff 15 e1 14 0a 00    	call   *0xa14e1(%rip)        # 16eb20 <_DYNAMIC+0x2a0>
   cd63f:	41 b6 01             	mov    $0x1,%r14b
   cd642:	48 8d 7c 24 40       	lea    0x40(%rsp),%rdi
   cd647:	e8 14 fb 03 00       	call   10d160 <<alloc::vec::into_iter::IntoIter<(hyperreal::rational::arithmetic::Rational, hyperreal::rational::arithmetic::Rational)> as core::ops::drop::Drop>::drop>
   cd64c:	31 db                	xor    %ebx,%ebx
   cd64e:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
   cd653:	4c 8d 68 18          	lea    0x18(%rax),%r13
   cd657:	48 8d bc 24 88 00 00 	lea    0x88(%rsp),%rdi
   cd65e:	00 
   cd65f:	e8 2c 36 fe ff       	call   b0c90 <core::ptr::drop_glue::<hyperreal::computable::node::RadicalCoefficient>>
   cd664:	48 8b 7c 24 20       	mov    0x20(%rsp),%rdi
   cd669:	48 8b 07             	mov    (%rdi),%rax
   cd66c:	f0 48 ff 08          	lock decq (%rax)
   cd670:	75 06                	jne    cd678 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x638>
   cd672:	ff 15 a8 14 0a 00    	call   *0xa14a8(%rip)        # 16eb20 <_DYNAMIC+0x2a0>
   cd678:	84 db                	test   %bl,%bl
   cd67a:	48 8b 5c 24 08       	mov    0x8(%rsp),%rbx
   cd67f:	74 23                	je     cd6a4 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x664>
   cd681:	48 89 df             	mov    %rbx,%rdi
   cd684:	e8 47 55 04 00       	call   112bd0 <<alloc::vec::Vec<(hyperreal::rational::arithmetic::Rational, hyperreal::rational::arithmetic::Rational)> as core::ops::drop::Drop>::drop>
   cd689:	48 8b 33             	mov    (%rbx),%rsi
   cd68c:	48 85 f6             	test   %rsi,%rsi
   cd68f:	74 13                	je     cd6a4 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x664>
   cd691:	48 8b 7b 08          	mov    0x8(%rbx),%rdi
   cd695:	48 c1 e6 04          	shl    $0x4,%rsi
   cd699:	ba 08 00 00 00       	mov    $0x8,%edx
   cd69e:	ff 15 9c 14 0a 00    	call   *0xa149c(%rip)        # 16eb40 <_DYNAMIC+0x2c0>
   cd6a4:	48 8b 7b 20          	mov    0x20(%rbx),%rdi
   cd6a8:	4c 8b 73 28          	mov    0x28(%rbx),%r14
   cd6ac:	49 ff c6             	inc    %r14
   cd6af:	90                   	nop
   cd6b0:	49 83 fe 01          	cmp    $0x1,%r14
   cd6b4:	74 11                	je     cd6c7 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x687>
   cd6b6:	48 8d 5f 18          	lea    0x18(%rdi),%rbx
   cd6ba:	49 ff ce             	dec    %r14
   cd6bd:	e8 9e 39 04 00       	call   111060 <core::ptr::drop_glue::<(hyperreal::computable::node::Computable, hyperreal::rational::arithmetic::Rational)>>
   cd6c2:	48 89 df             	mov    %rbx,%rdi
   cd6c5:	eb e9                	jmp    cd6b0 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x670>
   cd6c7:	49 8b 45 00          	mov    0x0(%r13),%rax
   cd6cb:	48 85 c0             	test   %rax,%rax
   cd6ce:	74 1c                	je     cd6ec <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x6ac>
   cd6d0:	48 8b 4c 24 08       	mov    0x8(%rsp),%rcx
   cd6d5:	48 8b 79 20          	mov    0x20(%rcx),%rdi
   cd6d9:	48 c1 e0 03          	shl    $0x3,%rax
   cd6dd:	48 8d 34 40          	lea    (%rax,%rax,2),%rsi
   cd6e1:	ba 08 00 00 00       	mov    $0x8,%edx
   cd6e6:	ff 15 54 14 0a 00    	call   *0xa1454(%rip)        # 16eb40 <_DYNAMIC+0x2c0>
   cd6ec:	48 81 c4 f8 00 00 00 	add    $0xf8,%rsp
   cd6f3:	5b                   	pop    %rbx
   cd6f4:	41 5c                	pop    %r12
   cd6f6:	41 5d                	pop    %r13
   cd6f8:	41 5e                	pop    %r14
   cd6fa:	41 5f                	pop    %r15
   cd6fc:	5d                   	pop    %rbp
   cd6fd:	c3                   	ret
   cd6fe:	0f 0b                	ud2
   cd700:	eb 34                	jmp    cd736 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x6f6>
   cd702:	eb 27                	jmp    cd72b <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x6eb>
   cd704:	eb 25                	jmp    cd72b <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x6eb>
   cd706:	eb 2e                	jmp    cd736 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x6f6>
   cd708:	4d 89 ee             	mov    %r13,%r14
   cd70b:	48 89 c5             	mov    %rax,%rbp
   cd70e:	84 db                	test   %bl,%bl
   cd710:	75 35                	jne    cd747 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x707>
   cd712:	e9 b9 00 00 00       	jmp    cd7d0 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x790>
   cd717:	48 89 c5             	mov    %rax,%rbp
   cd71a:	31 db                	xor    %ebx,%ebx
   cd71c:	44 89 74 24 04       	mov    %r14d,0x4(%rsp)
   cd721:	e9 01 02 00 00       	jmp    cd927 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x8e7>
   cd726:	48 89 c5             	mov    %rax,%rbp
   cd729:	eb 49                	jmp    cd774 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x734>
   cd72b:	48 89 c5             	mov    %rax,%rbp
   cd72e:	41 b6 01             	mov    $0x1,%r14b
   cd731:	e9 89 01 00 00       	jmp    cd8bf <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x87f>
   cd736:	48 89 c5             	mov    %rax,%rbp
   cd739:	41 b6 01             	mov    $0x1,%r14b
   cd73c:	e9 94 01 00 00       	jmp    cd8d5 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x895>
   cd741:	4d 89 ee             	mov    %r13,%r14
   cd744:	48 89 c5             	mov    %rax,%rbp
   cd747:	48 8b 7c 24 08       	mov    0x8(%rsp),%rdi
   cd74c:	e8 0f 22 fe ff       	call   af960 <core::ptr::drop_glue::<alloc::vec::Vec<(hyperreal::rational::arithmetic::Rational, hyperreal::rational::arithmetic::Rational)>>>
   cd751:	eb 7d                	jmp    cd7d0 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x790>
   cd753:	48 89 c5             	mov    %rax,%rbp
   cd756:	e9 dd 00 00 00       	jmp    cd838 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x7f8>
   cd75b:	48 89 c5             	mov    %rax,%rbp
   cd75e:	eb 1e                	jmp    cd77e <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x73e>
   cd760:	48 89 c5             	mov    %rax,%rbp
   cd763:	f0 48 ff 0b          	lock decq (%rbx)
   cd767:	75 0b                	jne    cd774 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x734>
   cd769:	48 8d 7c 24 38       	lea    0x38(%rsp),%rdi
   cd76e:	ff 15 ac 13 0a 00    	call   *0xa13ac(%rip)        # 16eb20 <_DYNAMIC+0x2a0>
   cd774:	48 8d 7c 24 68       	lea    0x68(%rsp),%rdi
   cd779:	e8 72 30 fe ff       	call   b07f0 <core::ptr::drop_glue::<hyperreal::computable::node::Computable>>
   cd77e:	c7 44 24 04 00 00 00 	movl   $0x0,0x4(%rsp)
   cd785:	00 
   cd786:	48 8d 7c 24 40       	lea    0x40(%rsp),%rdi
   cd78b:	e8 00 f9 03 00       	call   10d090 <<alloc::vec::into_iter::IntoIter<(hyperreal::computable::node::Computable, hyperreal::rational::arithmetic::Rational)> as core::ops::drop::Drop>::drop>
   cd790:	31 db                	xor    %ebx,%ebx
   cd792:	e9 90 01 00 00       	jmp    cd927 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x8e7>
   cd797:	ff 15 8b 13 0a 00    	call   *0xa138b(%rip)        # 16eb28 <_DYNAMIC+0x2a8>
   cd79d:	4c 89 6c 24 60       	mov    %r13,0x60(%rsp)
   cd7a2:	48 89 c5             	mov    %rax,%rbp
   cd7a5:	e9 b3 01 00 00       	jmp    cd95d <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x91d>
   cd7aa:	4d 89 ee             	mov    %r13,%r14
   cd7ad:	48 89 c5             	mov    %rax,%rbp
   cd7b0:	48 8b 33             	mov    (%rbx),%rsi
   cd7b3:	48 85 f6             	test   %rsi,%rsi
   cd7b6:	74 18                	je     cd7d0 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x790>
   cd7b8:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
   cd7bd:	48 8b 78 08          	mov    0x8(%rax),%rdi
   cd7c1:	48 c1 e6 04          	shl    $0x4,%rsi
   cd7c5:	ba 08 00 00 00       	mov    $0x8,%edx
   cd7ca:	ff 15 70 13 0a 00    	call   *0xa1370(%rip)        # 16eb40 <_DYNAMIC+0x2c0>
   cd7d0:	4c 89 f7             	mov    %r14,%rdi
   cd7d3:	e8 b8 20 fe ff       	call   af890 <core::ptr::drop_glue::<alloc::vec::Vec<(hyperreal::computable::node::Computable, hyperreal::rational::arithmetic::Rational)>>>
   cd7d8:	e9 fe 01 00 00       	jmp    cd9db <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x99b>
   cd7dd:	48 89 c5             	mov    %rax,%rbp
   cd7e0:	48 8d 7c 24 40       	lea    0x40(%rsp),%rdi
   cd7e5:	e8 06 30 fe ff       	call   b07f0 <core::ptr::drop_glue::<hyperreal::computable::node::Computable>>
   cd7ea:	eb 03                	jmp    cd7ef <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x7af>
   cd7ec:	48 89 c5             	mov    %rax,%rbp
   cd7ef:	c7 44 24 04 00 00 00 	movl   $0x0,0x4(%rsp)
   cd7f6:	00 
   cd7f7:	31 db                	xor    %ebx,%ebx
   cd7f9:	e9 29 01 00 00       	jmp    cd927 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x8e7>
   cd7fe:	48 89 c5             	mov    %rax,%rbp
   cd801:	f0 49 ff 0c 24       	lock decq (%r12)
   cd806:	41 b6 01             	mov    $0x1,%r14b
   cd809:	0f 85 9a 00 00 00    	jne    cd8a9 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x869>
   cd80f:	48 8d 7c 24 68       	lea    0x68(%rsp),%rdi
   cd814:	ff 15 06 13 0a 00    	call   *0xa1306(%rip)        # 16eb20 <_DYNAMIC+0x2a0>
   cd81a:	e9 8a 00 00 00       	jmp    cd8a9 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x869>
   cd81f:	48 89 c5             	mov    %rax,%rbp
   cd822:	eb 26                	jmp    cd84a <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x80a>
   cd824:	48 89 c5             	mov    %rax,%rbp
   cd827:	f0 48 ff 0b          	lock decq (%rbx)
   cd82b:	75 0b                	jne    cd838 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x7f8>
   cd82d:	48 8d 7c 24 68       	lea    0x68(%rsp),%rdi
   cd832:	ff 15 e8 12 0a 00    	call   *0xa12e8(%rip)        # 16eb20 <_DYNAMIC+0x2a0>
   cd838:	f0 49 ff 0c 24       	lock decq (%r12)
   cd83d:	75 0b                	jne    cd84a <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x80a>
   cd83f:	48 8d 7c 24 38       	lea    0x38(%rsp),%rdi
   cd844:	ff 15 d6 12 0a 00    	call   *0xa12d6(%rip)        # 16eb20 <_DYNAMIC+0x2a0>
   cd84a:	45 31 f6             	xor    %r14d,%r14d
   cd84d:	eb 5a                	jmp    cd8a9 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x869>
   cd84f:	ff 15 d3 12 0a 00    	call   *0xa12d3(%rip)        # 16eb28 <_DYNAMIC+0x2a8>
   cd855:	eb 00                	jmp    cd857 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x817>
   cd857:	48 89 c5             	mov    %rax,%rbp
   cd85a:	41 b6 01             	mov    $0x1,%r14b
   cd85d:	eb 4a                	jmp    cd8a9 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x869>
   cd85f:	48 89 c5             	mov    %rax,%rbp
   cd862:	f0 49 ff 0f          	lock decq (%r15)
   cd866:	b0 01                	mov    $0x1,%al
   cd868:	89 44 24 04          	mov    %eax,0x4(%rsp)
   cd86c:	0f 85 b3 00 00 00    	jne    cd925 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x8e5>
   cd872:	b0 01                	mov    $0x1,%al
   cd874:	89 44 24 04          	mov    %eax,0x4(%rsp)
   cd878:	48 8d 7c 24 40       	lea    0x40(%rsp),%rdi
   cd87d:	ff 15 9d 12 0a 00    	call   *0xa129d(%rip)        # 16eb20 <_DYNAMIC+0x2a0>
   cd883:	b3 01                	mov    $0x1,%bl
   cd885:	e9 9d 00 00 00       	jmp    cd927 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x8e7>
   cd88a:	e9 8d 00 00 00       	jmp    cd91c <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x8dc>
   cd88f:	48 89 c5             	mov    %rax,%rbp
   cd892:	f0 48 ff 0b          	lock decq (%rbx)
   cd896:	41 b6 01             	mov    $0x1,%r14b
   cd899:	75 0e                	jne    cd8a9 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x869>
   cd89b:	48 8d bc 24 f0 00 00 	lea    0xf0(%rsp),%rdi
   cd8a2:	00 
   cd8a3:	ff 15 77 12 0a 00    	call   *0xa1277(%rip)        # 16eb20 <_DYNAMIC+0x2a0>
   cd8a9:	48 8b 44 24 78       	mov    0x78(%rsp),%rax
   cd8ae:	f0 48 ff 08          	lock decq (%rax)
   cd8b2:	75 0b                	jne    cd8bf <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x87f>
   cd8b4:	48 8d 7c 24 28       	lea    0x28(%rsp),%rdi
   cd8b9:	ff 15 61 12 0a 00    	call   *0xa1261(%rip)        # 16eb20 <_DYNAMIC+0x2a0>
   cd8bf:	48 8b 44 24 30       	mov    0x30(%rsp),%rax
   cd8c4:	f0 48 ff 08          	lock decq (%rax)
   cd8c8:	75 0b                	jne    cd8d5 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x895>
   cd8ca:	48 8d 7c 24 10       	lea    0x10(%rsp),%rdi
   cd8cf:	ff 15 4b 12 0a 00    	call   *0xa124b(%rip)        # 16eb20 <_DYNAMIC+0x2a0>
   cd8d5:	48 8d 7c 24 40       	lea    0x40(%rsp),%rdi
   cd8da:	e8 81 f8 03 00       	call   10d160 <<alloc::vec::into_iter::IntoIter<(hyperreal::rational::arithmetic::Rational, hyperreal::rational::arithmetic::Rational)> as core::ops::drop::Drop>::drop>
   cd8df:	b0 01                	mov    $0x1,%al
   cd8e1:	89 44 24 04          	mov    %eax,0x4(%rsp)
   cd8e5:	31 db                	xor    %ebx,%ebx
   cd8e7:	45 84 f6             	test   %r14b,%r14b
   cd8ea:	75 3b                	jne    cd927 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x8e7>
   cd8ec:	eb 55                	jmp    cd943 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x903>
   cd8ee:	4c 89 6c 24 60       	mov    %r13,0x60(%rsp)
   cd8f3:	48 89 c5             	mov    %rax,%rbp
   cd8f6:	f0 49 ff 0f          	lock decq (%r15)
   cd8fa:	b0 01                	mov    $0x1,%al
   cd8fc:	89 44 24 04          	mov    %eax,0x4(%rsp)
   cd900:	75 23                	jne    cd925 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x8e5>
   cd902:	b0 01                	mov    $0x1,%al
   cd904:	89 44 24 04          	mov    %eax,0x4(%rsp)
   cd908:	48 8d 7c 24 40       	lea    0x40(%rsp),%rdi
   cd90d:	ff 15 0d 12 0a 00    	call   *0xa120d(%rip)        # 16eb20 <_DYNAMIC+0x2a0>
   cd913:	b3 01                	mov    $0x1,%bl
   cd915:	eb 10                	jmp    cd927 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x8e7>
   cd917:	4c 89 6c 24 60       	mov    %r13,0x60(%rsp)
   cd91c:	48 89 c5             	mov    %rax,%rbp
   cd91f:	b0 01                	mov    $0x1,%al
   cd921:	89 44 24 04          	mov    %eax,0x4(%rsp)
   cd925:	b3 01                	mov    $0x1,%bl
   cd927:	48 8b 84 24 b8 00 00 	mov    0xb8(%rsp),%rax
   cd92e:	00 
   cd92f:	f0 48 ff 08          	lock decq (%rax)
   cd933:	75 0e                	jne    cd943 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x903>
   cd935:	48 8d bc 24 b8 00 00 	lea    0xb8(%rsp),%rdi
   cd93c:	00 
   cd93d:	ff 15 dd 11 0a 00    	call   *0xa11dd(%rip)        # 16eb20 <_DYNAMIC+0x2a0>
   cd943:	48 8d bc 24 88 00 00 	lea    0x88(%rsp),%rdi
   cd94a:	00 
   cd94b:	e8 10 20 fe ff       	call   af960 <core::ptr::drop_glue::<alloc::vec::Vec<(hyperreal::rational::arithmetic::Rational, hyperreal::rational::arithmetic::Rational)>>>
   cd950:	48 8d bc 24 a0 00 00 	lea    0xa0(%rsp),%rdi
   cd957:	00 
   cd958:	e8 33 1f fe ff       	call   af890 <core::ptr::drop_glue::<alloc::vec::Vec<(hyperreal::computable::node::Computable, hyperreal::rational::arithmetic::Rational)>>>
   cd95d:	48 8b 44 24 20       	mov    0x20(%rsp),%rax
   cd962:	48 8b 00             	mov    (%rax),%rax
   cd965:	f0 48 ff 08          	lock decq (%rax)
   cd969:	75 0b                	jne    cd976 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x936>
   cd96b:	48 8b 7c 24 20       	mov    0x20(%rsp),%rdi
   cd970:	ff 15 aa 11 0a 00    	call   *0xa11aa(%rip)        # 16eb20 <_DYNAMIC+0x2a0>
   cd976:	84 db                	test   %bl,%bl
   cd978:	74 0a                	je     cd984 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x944>
   cd97a:	48 8b 7c 24 08       	mov    0x8(%rsp),%rdi
   cd97f:	e8 dc 1f fe ff       	call   af960 <core::ptr::drop_glue::<alloc::vec::Vec<(hyperreal::rational::arithmetic::Rational, hyperreal::rational::arithmetic::Rational)>>>
   cd984:	80 7c 24 04 00       	cmpb   $0x0,0x4(%rsp)
   cd989:	74 50                	je     cd9db <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x99b>
   cd98b:	48 8b 7c 24 60       	mov    0x60(%rsp),%rdi
   cd990:	e8 fb 1e fe ff       	call   af890 <core::ptr::drop_glue::<alloc::vec::Vec<(hyperreal::computable::node::Computable, hyperreal::rational::arithmetic::Rational)>>>
   cd995:	eb 44                	jmp    cd9db <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x99b>
   cd997:	ff 15 8b 11 0a 00    	call   *0xa118b(%rip)        # 16eb28 <_DYNAMIC+0x2a8>
   cd99d:	48 89 c5             	mov    %rax,%rbp
   cd9a0:	49 ff ce             	dec    %r14
   cd9a3:	74 11                	je     cd9b6 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x976>
   cd9a5:	4c 8d 7b 18          	lea    0x18(%rbx),%r15
   cd9a9:	48 89 df             	mov    %rbx,%rdi
   cd9ac:	e8 af 36 04 00       	call   111060 <core::ptr::drop_glue::<(hyperreal::computable::node::Computable, hyperreal::rational::arithmetic::Rational)>>
   cd9b1:	4c 89 fb             	mov    %r15,%rbx
   cd9b4:	eb ea                	jmp    cd9a0 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x960>
   cd9b6:	49 8b 45 00          	mov    0x0(%r13),%rax
   cd9ba:	48 85 c0             	test   %rax,%rax
   cd9bd:	74 1c                	je     cd9db <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x99b>
   cd9bf:	48 8b 4c 24 08       	mov    0x8(%rsp),%rcx
   cd9c4:	48 8b 79 20          	mov    0x20(%rcx),%rdi
   cd9c8:	48 c1 e0 03          	shl    $0x3,%rax
   cd9cc:	48 8d 34 40          	lea    (%rax,%rax,2),%rsi
   cd9d0:	ba 08 00 00 00       	mov    $0x8,%edx
   cd9d5:	ff 15 65 11 0a 00    	call   *0xa1165(%rip)        # 16eb40 <_DYNAMIC+0x2c0>
   cd9db:	48 89 ef             	mov    %rbp,%rdi
   cd9de:	e8 3d ae 09 00       	call   168820 <_Unwind_Resume@plt>
   cd9e3:	ff 15 3f 11 0a 00    	call   *0xa113f(%rip)        # 16eb28 <_DYNAMIC+0x2a8>
