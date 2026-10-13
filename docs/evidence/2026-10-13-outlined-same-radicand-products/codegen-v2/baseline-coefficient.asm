
/home/tim/Documents/GitHub/workspace/csgrs/target/outlined-same-radicand-products/cost-v2-binaries/baseline-quadratic-time:     file format elf64-x86-64


Disassembly of section .text:

00000000000cd050 <<hyperreal::computable::node::RadicalCoefficient>::multiply>:
   cd050:	55                   	push   %rbp
   cd051:	41 57                	push   %r15
   cd053:	41 56                	push   %r14
   cd055:	41 55                	push   %r13
   cd057:	41 54                	push   %r12
   cd059:	53                   	push   %rbx
   cd05a:	48 81 ec f8 00 00 00 	sub    $0xf8,%rsp
   cd061:	48 89 fd             	mov    %rdi,%rbp
   cd064:	4c 8d 6e 18          	lea    0x18(%rsi),%r13
   cd068:	48 89 74 24 08       	mov    %rsi,0x8(%rsp)
   cd06d:	48 83 7e 28 00       	cmpq   $0x0,0x28(%rsi)
   cd072:	74 10                	je     cd084 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x34>
   cd074:	48 83 7a 10 00       	cmpq   $0x0,0x10(%rdx)
   cd079:	75 1e                	jne    cd099 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x49>
   cd07b:	48 83 7a 28 00       	cmpq   $0x0,0x28(%rdx)
   cd080:	75 17                	jne    cd099 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x49>
   cd082:	eb 3f                	jmp    cd0c3 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x73>
   cd084:	48 8b 4a 28          	mov    0x28(%rdx),%rcx
   cd088:	48 85 c9             	test   %rcx,%rcx
   cd08b:	74 36                	je     cd0c3 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x73>
   cd08d:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
   cd092:	48 83 78 10 00       	cmpq   $0x0,0x10(%rax)
   cd097:	74 3e                	je     cd0d7 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x87>
   cd099:	48 c7 45 00 ff ff ff 	movq   $0xffffffffffffffff,0x0(%rbp)
   cd0a0:	ff 
   cd0a1:	48 8b 5c 24 08       	mov    0x8(%rsp),%rbx
   cd0a6:	48 8b 43 30          	mov    0x30(%rbx),%rax
   cd0aa:	f0 48 ff 08          	lock decq (%rax)
   cd0ae:	0f 85 dd 05 00 00    	jne    cd691 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x641>
   cd0b4:	48 8d 7b 30          	lea    0x30(%rbx),%rdi
   cd0b8:	ff 15 72 1a 0a 00    	call   *0xa1a72(%rip)        # 16eb30 <_DYNAMIC+0x2a0>
   cd0be:	e9 ce 05 00 00       	jmp    cd691 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x641>
   cd0c3:	b0 01                	mov    $0x1,%al
   cd0c5:	89 44 24 1c          	mov    %eax,0x1c(%rsp)
   cd0c9:	48 c7 84 24 80 00 00 	movq   $0x0,0x80(%rsp)
   cd0d0:	00 00 00 00 00 
   cd0d5:	eb 10                	jmp    cd0e7 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x97>
   cd0d7:	48 89 8c 24 80 00 00 	mov    %rcx,0x80(%rsp)
   cd0de:	00 
   cd0df:	c7 44 24 1c 00 00 00 	movl   $0x0,0x1c(%rsp)
   cd0e6:	00 
   cd0e7:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
   cd0ec:	48 8d 78 30          	lea    0x30(%rax),%rdi
   cd0f0:	48 89 94 24 c8 00 00 	mov    %rdx,0xc8(%rsp)
   cd0f7:	00 
   cd0f8:	48 8d 72 30          	lea    0x30(%rdx),%rsi
   cd0fc:	b0 01                	mov    $0x1,%al
   cd0fe:	89 44 24 04          	mov    %eax,0x4(%rsp)
   cd102:	b3 01                	mov    $0x1,%bl
   cd104:	48 89 7c 24 20       	mov    %rdi,0x20(%rsp)
   cd109:	48 89 b4 24 d0 00 00 	mov    %rsi,0xd0(%rsp)
   cd110:	00 
   cd111:	e8 8a 45 fc ff       	call   916a0 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cd116:	48 89 84 24 b8 00 00 	mov    %rax,0xb8(%rsp)
   cd11d:	00 
   cd11e:	48 c7 84 24 88 00 00 	movq   $0x0,0x88(%rsp)
   cd125:	00 00 00 00 00 
   cd12a:	48 c7 84 24 90 00 00 	movq   $0x8,0x90(%rsp)
   cd131:	00 08 00 00 00 
   cd136:	0f 57 c0             	xorps  %xmm0,%xmm0
   cd139:	0f 11 84 24 98 00 00 	movups %xmm0,0x98(%rsp)
   cd140:	00 
   cd141:	48 c7 84 24 a8 00 00 	movq   $0x8,0xa8(%rsp)
   cd148:	00 08 00 00 00 
   cd14d:	48 c7 84 24 b0 00 00 	movq   $0x0,0xb0(%rsp)
   cd154:	00 00 00 00 00 
   cd159:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
   cd15e:	4c 8b 70 08          	mov    0x8(%rax),%r14
   cd162:	4c 8b 60 10          	mov    0x10(%rax),%r12
   cd166:	49 c1 e4 04          	shl    $0x4,%r12
   cd16a:	48 8d 9c 24 88 00 00 	lea    0x88(%rsp),%rbx
   cd171:	00 
   cd172:	66 66 66 66 66 2e 0f 	data16 data16 data16 data16 cs nopw 0x0(%rax,%rax,1)
   cd179:	1f 84 00 00 00 00 00 
   cd180:	4d 85 e4             	test   %r12,%r12
   cd183:	74 4f                	je     cd1d4 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x184>
   cd185:	49 8b 06             	mov    (%r14),%rax
   cd188:	f0 48 ff 00          	lock incq (%rax)
   cd18c:	0f 8e 7c 05 00 00    	jle    cd70e <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x6be>
   cd192:	49 8d 7e 08          	lea    0x8(%r14),%rdi
   cd196:	4d 8b 3e             	mov    (%r14),%r15
   cd199:	4c 89 7c 24 40       	mov    %r15,0x40(%rsp)
   cd19e:	48 8b b4 24 d0 00 00 	mov    0xd0(%rsp),%rsi
   cd1a5:	00 
   cd1a6:	e8 f5 44 fc ff       	call   916a0 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cd1ab:	48 89 df             	mov    %rbx,%rdi
   cd1ae:	4c 89 fe             	mov    %r15,%rsi
   cd1b1:	48 89 c2             	mov    %rax,%rdx
   cd1b4:	e8 e7 f5 ff ff       	call   cc7a0 <<hyperreal::computable::node::RadicalCoefficient>::insert_radical>
   cd1b9:	49 83 c6 10          	add    $0x10,%r14
   cd1bd:	49 83 c4 f0          	add    $0xfffffffffffffff0,%r12
   cd1c1:	84 c0                	test   %al,%al
   cd1c3:	75 bb                	jne    cd180 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x130>
   cd1c5:	48 c7 45 00 ff ff ff 	movq   $0xffffffffffffffff,0x0(%rbp)
   cd1cc:	ff 
   cd1cd:	b3 01                	mov    $0x1,%bl
   cd1cf:	e9 93 04 00 00       	jmp    cd667 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x617>
   cd1d4:	4c 89 6c 24 60       	mov    %r13,0x60(%rsp)
   cd1d9:	48 8b 84 24 c8 00 00 	mov    0xc8(%rsp),%rax
   cd1e0:	00 
   cd1e1:	4c 8b 68 08          	mov    0x8(%rax),%r13
   cd1e5:	4c 8b 60 10          	mov    0x10(%rax),%r12
   cd1e9:	4d 89 e6             	mov    %r12,%r14
   cd1ec:	49 c1 e6 04          	shl    $0x4,%r14
   cd1f0:	48 8d 9c 24 88 00 00 	lea    0x88(%rsp),%rbx
   cd1f7:	00 
   cd1f8:	4c 89 b4 24 e0 00 00 	mov    %r14,0xe0(%rsp)
   cd1ff:	00 
   cd200:	4c 89 ac 24 e8 00 00 	mov    %r13,0xe8(%rsp)
   cd207:	00 
   cd208:	0f 1f 84 00 00 00 00 	nopl   0x0(%rax,%rax,1)
   cd20f:	00 
   cd210:	4d 85 f6             	test   %r14,%r14
   cd213:	74 4e                	je     cd263 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x213>
   cd215:	49 8b 45 00          	mov    0x0(%r13),%rax
   cd219:	f0 48 ff 00          	lock incq (%rax)
   cd21d:	0f 8e eb 04 00 00    	jle    cd70e <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x6be>
   cd223:	49 8d 7d 08          	lea    0x8(%r13),%rdi
   cd227:	4d 8b 7d 00          	mov    0x0(%r13),%r15
   cd22b:	4c 89 7c 24 40       	mov    %r15,0x40(%rsp)
   cd230:	48 8b 74 24 20       	mov    0x20(%rsp),%rsi
   cd235:	e8 66 44 fc ff       	call   916a0 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cd23a:	48 89 df             	mov    %rbx,%rdi
   cd23d:	4c 89 fe             	mov    %r15,%rsi
   cd240:	48 89 c2             	mov    %rax,%rdx
   cd243:	e8 58 f5 ff ff       	call   cc7a0 <<hyperreal::computable::node::RadicalCoefficient>::insert_radical>
   cd248:	49 83 c5 10          	add    $0x10,%r13
   cd24c:	49 83 c6 f0          	add    $0xfffffffffffffff0,%r14
   cd250:	84 c0                	test   %al,%al
   cd252:	75 bc                	jne    cd210 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x1c0>
   cd254:	48 c7 45 00 ff ff ff 	movq   $0xffffffffffffffff,0x0(%rbp)
   cd25b:	ff 
   cd25c:	b3 01                	mov    $0x1,%bl
   cd25e:	e9 fb 03 00 00       	jmp    cd65e <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x60e>
   cd263:	48 8b 4c 24 08       	mov    0x8(%rsp),%rcx
   cd268:	48 8b 01             	mov    (%rcx),%rax
   cd26b:	4c 8b 71 08          	mov    0x8(%rcx),%r14
   cd26f:	48 8b 49 10          	mov    0x10(%rcx),%rcx
   cd273:	49 89 cd             	mov    %rcx,%r13
   cd276:	49 c1 e5 04          	shl    $0x4,%r13
   cd27a:	4b 8d 14 2e          	lea    (%r14,%r13,1),%rdx
   cd27e:	4c 89 74 24 40       	mov    %r14,0x40(%rsp)
   cd283:	4c 89 74 24 48       	mov    %r14,0x48(%rsp)
   cd288:	48 89 44 24 50       	mov    %rax,0x50(%rsp)
   cd28d:	48 89 94 24 d8 00 00 	mov    %rdx,0xd8(%rsp)
   cd294:	00 
   cd295:	48 89 54 24 58       	mov    %rdx,0x58(%rsp)
   cd29a:	48 85 c9             	test   %rcx,%rcx
   cd29d:	48 89 ac 24 c0 00 00 	mov    %rbp,0xc0(%rsp)
   cd2a4:	00 
   cd2a5:	0f 84 c2 01 00 00    	je     cd46d <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x41d>
   cd2ab:	4d 85 e4             	test   %r12,%r12
   cd2ae:	75 77                	jne    cd327 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x2d7>
   cd2b0:	49 83 c6 10          	add    $0x10,%r14
   cd2b4:	48 8d 5c 24 28       	lea    0x28(%rsp),%rbx
   cd2b9:	4c 8b 25 70 18 0a 00 	mov    0xa1870(%rip),%r12        # 16eb30 <_DYNAMIC+0x2a0>
   cd2c0:	4c 8d 7c 24 10       	lea    0x10(%rsp),%r15
   cd2c5:	eb 17                	jmp    cd2de <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x28e>
   cd2c7:	66 0f 1f 84 00 00 00 	nopw   0x0(%rax,%rax,1)
   cd2ce:	00 00 
   cd2d0:	49 83 c6 10          	add    $0x10,%r14
   cd2d4:	49 83 c5 f0          	add    $0xfffffffffffffff0,%r13
   cd2d8:	0f 84 8f 01 00 00    	je     cd46d <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x41d>
   cd2de:	4c 89 74 24 48       	mov    %r14,0x48(%rsp)
   cd2e3:	49 8b 4e f0          	mov    -0x10(%r14),%rcx
   cd2e7:	49 8b 46 f8          	mov    -0x8(%r14),%rax
   cd2eb:	48 89 4c 24 30       	mov    %rcx,0x30(%rsp)
   cd2f0:	48 89 4c 24 10       	mov    %rcx,0x10(%rsp)
   cd2f5:	48 89 44 24 28       	mov    %rax,0x28(%rsp)
   cd2fa:	f0 48 ff 08          	lock decq (%rax)
   cd2fe:	75 06                	jne    cd306 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x2b6>
   cd300:	48 89 df             	mov    %rbx,%rdi
   cd303:	41 ff d4             	call   *%r12
   cd306:	48 8b 44 24 30       	mov    0x30(%rsp),%rax
   cd30b:	f0 48 ff 08          	lock decq (%rax)
   cd30f:	75 bf                	jne    cd2d0 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x280>
   cd311:	4c 89 ff             	mov    %r15,%rdi
   cd314:	41 ff d4             	call   *%r12
   cd317:	eb b7                	jmp    cd2d0 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x280>
   cd319:	4c 3b b4 24 d8 00 00 	cmp    0xd8(%rsp),%r14
   cd320:	00 
   cd321:	0f 84 46 01 00 00    	je     cd46d <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x41d>
   cd327:	4c 89 f0             	mov    %r14,%rax
   cd32a:	49 83 c6 10          	add    $0x10,%r14
   cd32e:	4c 89 74 24 48       	mov    %r14,0x48(%rsp)
   cd333:	48 8b 08             	mov    (%rax),%rcx
   cd336:	48 8b 40 08          	mov    0x8(%rax),%rax
   cd33a:	48 89 4c 24 30       	mov    %rcx,0x30(%rsp)
   cd33f:	48 89 4c 24 10       	mov    %rcx,0x10(%rsp)
   cd344:	48 89 44 24 78       	mov    %rax,0x78(%rsp)
   cd349:	48 89 44 24 28       	mov    %rax,0x28(%rsp)
   cd34e:	48 8b ac 24 e0 00 00 	mov    0xe0(%rsp),%rbp
   cd355:	00 
   cd356:	4c 8b bc 24 e8 00 00 	mov    0xe8(%rsp),%r15
   cd35d:	00 
   cd35e:	eb 0e                	jmp    cd36e <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x31e>
   cd360:	49 83 c7 10          	add    $0x10,%r15
   cd364:	48 83 c5 f0          	add    $0xfffffffffffffff0,%rbp
   cd368:	0f 84 ca 00 00 00    	je     cd438 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x3e8>
   cd36e:	49 8d 77 08          	lea    0x8(%r15),%rsi
   cd372:	48 8d 7c 24 28       	lea    0x28(%rsp),%rdi
   cd377:	e8 24 43 fc ff       	call   916a0 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cd37c:	48 89 c3             	mov    %rax,%rbx
   cd37f:	48 89 84 24 f0 00 00 	mov    %rax,0xf0(%rsp)
   cd386:	00 
   cd387:	48 8d 7c 24 10       	lea    0x10(%rsp),%rdi
   cd38c:	4c 89 fe             	mov    %r15,%rsi
   cd38f:	ff 15 43 17 0a 00    	call   *0xa1743(%rip)        # 16ead8 <_DYNAMIC+0x248>
   cd395:	84 c0                	test   %al,%al
   cd397:	74 57                	je     cd3f0 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x3a0>
   cd399:	4c 8b a4 24 b8 00 00 	mov    0xb8(%rsp),%r12
   cd3a0:	00 
   cd3a1:	4c 89 64 24 38       	mov    %r12,0x38(%rsp)
   cd3a6:	48 89 5c 24 68       	mov    %rbx,0x68(%rsp)
   cd3ab:	48 8d 7c 24 68       	lea    0x68(%rsp),%rdi
   cd3b0:	48 8d 74 24 10       	lea    0x10(%rsp),%rsi
   cd3b5:	e8 e6 42 fc ff       	call   916a0 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cd3ba:	49 89 c5             	mov    %rax,%r13
   cd3bd:	f0 48 ff 0b          	lock decq (%rbx)
   cd3c1:	75 0b                	jne    cd3ce <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x37e>
   cd3c3:	48 8d 7c 24 68       	lea    0x68(%rsp),%rdi
   cd3c8:	ff 15 62 17 0a 00    	call   *0xa1762(%rip)        # 16eb30 <_DYNAMIC+0x2a0>
   cd3ce:	4c 89 e7             	mov    %r12,%rdi
   cd3d1:	4c 89 ee             	mov    %r13,%rsi
   cd3d4:	e8 97 29 fc ff       	call   8fd70 <<hyperreal::rational::arithmetic::Rational as core::ops::arith::Add>::add>
   cd3d9:	48 89 84 24 b8 00 00 	mov    %rax,0xb8(%rsp)
   cd3e0:	00 
   cd3e1:	e9 7a ff ff ff       	jmp    cd360 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x310>
   cd3e6:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
   cd3ed:	00 00 00 
   cd3f0:	48 8d 7c 24 10       	lea    0x10(%rsp),%rdi
   cd3f5:	4c 89 fe             	mov    %r15,%rsi
   cd3f8:	e8 a3 42 fc ff       	call   916a0 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cd3fd:	48 89 c7             	mov    %rax,%rdi
   cd400:	e8 6b 5e fa ff       	call   73270 <<hyperreal::rational::arithmetic::Rational>::extract_square_reduced_retained>
   cd405:	49 89 d4             	mov    %rdx,%r12
   cd408:	48 89 54 24 68       	mov    %rdx,0x68(%rsp)
   cd40d:	48 89 df             	mov    %rbx,%rdi
   cd410:	48 89 c6             	mov    %rax,%rsi
   cd413:	e8 08 4e fc ff       	call   92220 <<hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cd418:	48 8d bc 24 88 00 00 	lea    0x88(%rsp),%rdi
   cd41f:	00 
   cd420:	4c 89 e6             	mov    %r12,%rsi
   cd423:	48 89 c2             	mov    %rax,%rdx
   cd426:	e8 75 f3 ff ff       	call   cc7a0 <<hyperreal::computable::node::RadicalCoefficient>::insert_radical>
   cd42b:	84 c0                	test   %al,%al
   cd42d:	0f 85 2d ff ff ff    	jne    cd360 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x310>
   cd433:	e9 dc 01 00 00       	jmp    cd614 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x5c4>
   cd438:	48 8b 44 24 78       	mov    0x78(%rsp),%rax
   cd43d:	f0 48 ff 08          	lock decq (%rax)
   cd441:	75 0b                	jne    cd44e <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x3fe>
   cd443:	48 8d 7c 24 28       	lea    0x28(%rsp),%rdi
   cd448:	ff 15 e2 16 0a 00    	call   *0xa16e2(%rip)        # 16eb30 <_DYNAMIC+0x2a0>
   cd44e:	48 8b 44 24 30       	mov    0x30(%rsp),%rax
   cd453:	f0 48 ff 08          	lock decq (%rax)
   cd457:	0f 85 bc fe ff ff    	jne    cd319 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x2c9>
   cd45d:	48 8d 7c 24 10       	lea    0x10(%rsp),%rdi
   cd462:	ff 15 c8 16 0a 00    	call   *0xa16c8(%rip)        # 16eb30 <_DYNAMIC+0x2a0>
   cd468:	e9 ac fe ff ff       	jmp    cd319 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x2c9>
   cd46d:	41 b6 01             	mov    $0x1,%r14b
   cd470:	48 8d 7c 24 40       	lea    0x40(%rsp),%rdi
   cd475:	e8 f6 fc 03 00       	call   10d170 <<alloc::vec::into_iter::IntoIter<(hyperreal::rational::arithmetic::Rational, hyperreal::rational::arithmetic::Rational)> as core::ops::drop::Drop>::drop>
   cd47a:	48 8b 4c 24 08       	mov    0x8(%rsp),%rcx
   cd47f:	48 8b 41 18          	mov    0x18(%rcx),%rax
   cd483:	4c 8b 71 20          	mov    0x20(%rcx),%r14
   cd487:	48 8b 49 28          	mov    0x28(%rcx),%rcx
   cd48b:	48 8d 14 cd 00 00 00 	lea    0x0(,%rcx,8),%rdx
   cd492:	00 
   cd493:	4c 8d 2c 52          	lea    (%rdx,%rdx,2),%r13
   cd497:	4b 8d 14 2e          	lea    (%r14,%r13,1),%rdx
   cd49b:	4c 89 74 24 40       	mov    %r14,0x40(%rsp)
   cd4a0:	4c 89 74 24 48       	mov    %r14,0x48(%rsp)
   cd4a5:	48 89 44 24 50       	mov    %rax,0x50(%rsp)
   cd4aa:	48 89 54 24 58       	mov    %rdx,0x58(%rsp)
   cd4af:	48 85 c9             	test   %rcx,%rcx
   cd4b2:	74 77                	je     cd52b <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x4db>
   cd4b4:	49 83 c6 18          	add    $0x18,%r14
   cd4b8:	0f 1f 84 00 00 00 00 	nopl   0x0(%rax,%rax,1)
   cd4bf:	00 
   cd4c0:	4c 89 74 24 48       	mov    %r14,0x48(%rsp)
   cd4c5:	4d 8b 66 e8          	mov    -0x18(%r14),%r12
   cd4c9:	4d 85 e4             	test   %r12,%r12
   cd4cc:	74 5d                	je     cd52b <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x4db>
   cd4ce:	49 8b 6e f0          	mov    -0x10(%r14),%rbp
   cd4d2:	49 8b 5e f8          	mov    -0x8(%r14),%rbx
   cd4d6:	4c 89 64 24 68       	mov    %r12,0x68(%rsp)
   cd4db:	48 89 6c 24 70       	mov    %rbp,0x70(%rsp)
   cd4e0:	48 89 5c 24 38       	mov    %rbx,0x38(%rsp)
   cd4e5:	48 8d 7c 24 38       	lea    0x38(%rsp),%rdi
   cd4ea:	48 8b b4 24 d0 00 00 	mov    0xd0(%rsp),%rsi
   cd4f1:	00 
   cd4f2:	e8 a9 41 fc ff       	call   916a0 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cd4f7:	49 89 c7             	mov    %rax,%r15
   cd4fa:	f0 48 ff 0b          	lock decq (%rbx)
   cd4fe:	75 0b                	jne    cd50b <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x4bb>
   cd500:	48 8d 7c 24 38       	lea    0x38(%rsp),%rdi
   cd505:	ff 15 25 16 0a 00    	call   *0xa1625(%rip)        # 16eb30 <_DYNAMIC+0x2a0>
   cd50b:	48 8d bc 24 88 00 00 	lea    0x88(%rsp),%rdi
   cd512:	00 
   cd513:	4c 89 e6             	mov    %r12,%rsi
   cd516:	48 89 ea             	mov    %rbp,%rdx
   cd519:	4c 89 f9             	mov    %r15,%rcx
   cd51c:	e8 2f ef ff ff       	call   cc450 <<hyperreal::computable::node::RadicalCoefficient>::insert_atom>
   cd521:	49 83 c6 18          	add    $0x18,%r14
   cd525:	49 83 c5 e8          	add    $0xffffffffffffffe8,%r13
   cd529:	75 95                	jne    cd4c0 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x470>
   cd52b:	45 31 f6             	xor    %r14d,%r14d
   cd52e:	48 8d 7c 24 40       	lea    0x40(%rsp),%rdi
   cd533:	e8 68 fb 03 00       	call   10d0a0 <<alloc::vec::into_iter::IntoIter<(hyperreal::computable::node::Computable, hyperreal::rational::arithmetic::Rational)> as core::ops::drop::Drop>::drop>
   cd538:	80 7c 24 1c 00       	cmpb   $0x0,0x1c(%rsp)
   cd53d:	48 8b ac 24 c0 00 00 	mov    0xc0(%rsp),%rbp
   cd544:	00 
   cd545:	0f 85 7c 00 00 00    	jne    cd5c7 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x577>
   cd54b:	48 8b 84 24 c8 00 00 	mov    0xc8(%rsp),%rax
   cd552:	00 
   cd553:	48 8b 58 20          	mov    0x20(%rax),%rbx
   cd557:	48 8b 84 24 80 00 00 	mov    0x80(%rsp),%rax
   cd55e:	00 
   cd55f:	48 8d 04 40          	lea    (%rax,%rax,2),%rax
   cd563:	4c 8d 34 c3          	lea    (%rbx,%rax,8),%r14
   cd567:	4c 8d bc 24 88 00 00 	lea    0x88(%rsp),%r15
   cd56e:	00 
   cd56f:	90                   	nop
   cd570:	4c 8b 23             	mov    (%rbx),%r12
   cd573:	4c 8b 6b 08          	mov    0x8(%rbx),%r13
   cd577:	f0 49 ff 04 24       	lock incq (%r12)
   cd57c:	0f 8e 8c 01 00 00    	jle    cd70e <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x6be>
   cd582:	4d 85 ed             	test   %r13,%r13
   cd585:	48 8b 74 24 20       	mov    0x20(%rsp),%rsi
   cd58a:	74 0b                	je     cd597 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x547>
   cd58c:	f0 49 ff 45 00       	lock incq 0x0(%r13)
   cd591:	0f 8e 77 01 00 00    	jle    cd70e <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x6be>
   cd597:	48 83 c3 10          	add    $0x10,%rbx
   cd59b:	4c 89 64 24 40       	mov    %r12,0x40(%rsp)
   cd5a0:	4c 89 6c 24 48       	mov    %r13,0x48(%rsp)
   cd5a5:	48 89 df             	mov    %rbx,%rdi
   cd5a8:	e8 f3 40 fc ff       	call   916a0 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cd5ad:	4c 89 ff             	mov    %r15,%rdi
   cd5b0:	4c 89 e6             	mov    %r12,%rsi
   cd5b3:	4c 89 ea             	mov    %r13,%rdx
   cd5b6:	48 89 c1             	mov    %rax,%rcx
   cd5b9:	e8 92 ee ff ff       	call   cc450 <<hyperreal::computable::node::RadicalCoefficient>::insert_atom>
   cd5be:	48 83 c3 08          	add    $0x8,%rbx
   cd5c2:	4c 39 f3             	cmp    %r14,%rbx
   cd5c5:	75 a9                	jne    cd570 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x520>
   cd5c7:	48 8b 84 24 b8 00 00 	mov    0xb8(%rsp),%rax
   cd5ce:	00 
   cd5cf:	48 89 45 30          	mov    %rax,0x30(%rbp)
   cd5d3:	0f 10 84 24 88 00 00 	movups 0x88(%rsp),%xmm0
   cd5da:	00 
   cd5db:	0f 10 8c 24 98 00 00 	movups 0x98(%rsp),%xmm1
   cd5e2:	00 
   cd5e3:	0f 10 94 24 a8 00 00 	movups 0xa8(%rsp),%xmm2
   cd5ea:	00 
   cd5eb:	0f 11 55 20          	movups %xmm2,0x20(%rbp)
   cd5ef:	0f 11 4d 10          	movups %xmm1,0x10(%rbp)
   cd5f3:	0f 11 45 00          	movups %xmm0,0x0(%rbp)
   cd5f7:	48 8b 7c 24 20       	mov    0x20(%rsp),%rdi
   cd5fc:	48 8b 07             	mov    (%rdi),%rax
   cd5ff:	f0 48 ff 08          	lock decq (%rax)
   cd603:	0f 85 f3 00 00 00    	jne    cd6fc <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x6ac>
   cd609:	ff 15 21 15 0a 00    	call   *0xa1521(%rip)        # 16eb30 <_DYNAMIC+0x2a0>
   cd60f:	e9 e8 00 00 00       	jmp    cd6fc <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x6ac>
   cd614:	48 8b 84 24 c0 00 00 	mov    0xc0(%rsp),%rax
   cd61b:	00 
   cd61c:	48 c7 00 ff ff ff ff 	movq   $0xffffffffffffffff,(%rax)
   cd623:	48 8b 44 24 78       	mov    0x78(%rsp),%rax
   cd628:	f0 48 ff 08          	lock decq (%rax)
   cd62c:	75 0b                	jne    cd639 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x5e9>
   cd62e:	48 8d 7c 24 28       	lea    0x28(%rsp),%rdi
   cd633:	ff 15 f7 14 0a 00    	call   *0xa14f7(%rip)        # 16eb30 <_DYNAMIC+0x2a0>
   cd639:	48 8b 44 24 30       	mov    0x30(%rsp),%rax
   cd63e:	f0 48 ff 08          	lock decq (%rax)
   cd642:	75 0b                	jne    cd64f <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x5ff>
   cd644:	48 8d 7c 24 10       	lea    0x10(%rsp),%rdi
   cd649:	ff 15 e1 14 0a 00    	call   *0xa14e1(%rip)        # 16eb30 <_DYNAMIC+0x2a0>
   cd64f:	41 b6 01             	mov    $0x1,%r14b
   cd652:	48 8d 7c 24 40       	lea    0x40(%rsp),%rdi
   cd657:	e8 14 fb 03 00       	call   10d170 <<alloc::vec::into_iter::IntoIter<(hyperreal::rational::arithmetic::Rational, hyperreal::rational::arithmetic::Rational)> as core::ops::drop::Drop>::drop>
   cd65c:	31 db                	xor    %ebx,%ebx
   cd65e:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
   cd663:	4c 8d 68 18          	lea    0x18(%rax),%r13
   cd667:	48 8d bc 24 88 00 00 	lea    0x88(%rsp),%rdi
   cd66e:	00 
   cd66f:	e8 2c 36 fe ff       	call   b0ca0 <core::ptr::drop_glue::<hyperreal::computable::node::RadicalCoefficient>>
   cd674:	48 8b 7c 24 20       	mov    0x20(%rsp),%rdi
   cd679:	48 8b 07             	mov    (%rdi),%rax
   cd67c:	f0 48 ff 08          	lock decq (%rax)
   cd680:	75 06                	jne    cd688 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x638>
   cd682:	ff 15 a8 14 0a 00    	call   *0xa14a8(%rip)        # 16eb30 <_DYNAMIC+0x2a0>
   cd688:	84 db                	test   %bl,%bl
   cd68a:	48 8b 5c 24 08       	mov    0x8(%rsp),%rbx
   cd68f:	74 23                	je     cd6b4 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x664>
   cd691:	48 89 df             	mov    %rbx,%rdi
   cd694:	e8 47 55 04 00       	call   112be0 <<alloc::vec::Vec<(hyperreal::rational::arithmetic::Rational, hyperreal::rational::arithmetic::Rational)> as core::ops::drop::Drop>::drop>
   cd699:	48 8b 33             	mov    (%rbx),%rsi
   cd69c:	48 85 f6             	test   %rsi,%rsi
   cd69f:	74 13                	je     cd6b4 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x664>
   cd6a1:	48 8b 7b 08          	mov    0x8(%rbx),%rdi
   cd6a5:	48 c1 e6 04          	shl    $0x4,%rsi
   cd6a9:	ba 08 00 00 00       	mov    $0x8,%edx
   cd6ae:	ff 15 9c 14 0a 00    	call   *0xa149c(%rip)        # 16eb50 <_DYNAMIC+0x2c0>
   cd6b4:	48 8b 7b 20          	mov    0x20(%rbx),%rdi
   cd6b8:	4c 8b 73 28          	mov    0x28(%rbx),%r14
   cd6bc:	49 ff c6             	inc    %r14
   cd6bf:	90                   	nop
   cd6c0:	49 83 fe 01          	cmp    $0x1,%r14
   cd6c4:	74 11                	je     cd6d7 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x687>
   cd6c6:	48 8d 5f 18          	lea    0x18(%rdi),%rbx
   cd6ca:	49 ff ce             	dec    %r14
   cd6cd:	e8 9e 39 04 00       	call   111070 <core::ptr::drop_glue::<(hyperreal::computable::node::Computable, hyperreal::rational::arithmetic::Rational)>>
   cd6d2:	48 89 df             	mov    %rbx,%rdi
   cd6d5:	eb e9                	jmp    cd6c0 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x670>
   cd6d7:	49 8b 45 00          	mov    0x0(%r13),%rax
   cd6db:	48 85 c0             	test   %rax,%rax
   cd6de:	74 1c                	je     cd6fc <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x6ac>
   cd6e0:	48 8b 4c 24 08       	mov    0x8(%rsp),%rcx
   cd6e5:	48 8b 79 20          	mov    0x20(%rcx),%rdi
   cd6e9:	48 c1 e0 03          	shl    $0x3,%rax
   cd6ed:	48 8d 34 40          	lea    (%rax,%rax,2),%rsi
   cd6f1:	ba 08 00 00 00       	mov    $0x8,%edx
   cd6f6:	ff 15 54 14 0a 00    	call   *0xa1454(%rip)        # 16eb50 <_DYNAMIC+0x2c0>
   cd6fc:	48 81 c4 f8 00 00 00 	add    $0xf8,%rsp
   cd703:	5b                   	pop    %rbx
   cd704:	41 5c                	pop    %r12
   cd706:	41 5d                	pop    %r13
   cd708:	41 5e                	pop    %r14
   cd70a:	41 5f                	pop    %r15
   cd70c:	5d                   	pop    %rbp
   cd70d:	c3                   	ret
   cd70e:	0f 0b                	ud2
   cd710:	eb 34                	jmp    cd746 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x6f6>
   cd712:	eb 27                	jmp    cd73b <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x6eb>
   cd714:	eb 25                	jmp    cd73b <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x6eb>
   cd716:	eb 2e                	jmp    cd746 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x6f6>
   cd718:	4d 89 ee             	mov    %r13,%r14
   cd71b:	48 89 c5             	mov    %rax,%rbp
   cd71e:	84 db                	test   %bl,%bl
   cd720:	75 35                	jne    cd757 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x707>
   cd722:	e9 b9 00 00 00       	jmp    cd7e0 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x790>
   cd727:	48 89 c5             	mov    %rax,%rbp
   cd72a:	31 db                	xor    %ebx,%ebx
   cd72c:	44 89 74 24 04       	mov    %r14d,0x4(%rsp)
   cd731:	e9 01 02 00 00       	jmp    cd937 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x8e7>
   cd736:	48 89 c5             	mov    %rax,%rbp
   cd739:	eb 49                	jmp    cd784 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x734>
   cd73b:	48 89 c5             	mov    %rax,%rbp
   cd73e:	41 b6 01             	mov    $0x1,%r14b
   cd741:	e9 89 01 00 00       	jmp    cd8cf <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x87f>
   cd746:	48 89 c5             	mov    %rax,%rbp
   cd749:	41 b6 01             	mov    $0x1,%r14b
   cd74c:	e9 94 01 00 00       	jmp    cd8e5 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x895>
   cd751:	4d 89 ee             	mov    %r13,%r14
   cd754:	48 89 c5             	mov    %rax,%rbp
   cd757:	48 8b 7c 24 08       	mov    0x8(%rsp),%rdi
   cd75c:	e8 0f 22 fe ff       	call   af970 <core::ptr::drop_glue::<alloc::vec::Vec<(hyperreal::rational::arithmetic::Rational, hyperreal::rational::arithmetic::Rational)>>>
   cd761:	eb 7d                	jmp    cd7e0 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x790>
   cd763:	48 89 c5             	mov    %rax,%rbp
   cd766:	e9 dd 00 00 00       	jmp    cd848 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x7f8>
   cd76b:	48 89 c5             	mov    %rax,%rbp
   cd76e:	eb 1e                	jmp    cd78e <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x73e>
   cd770:	48 89 c5             	mov    %rax,%rbp
   cd773:	f0 48 ff 0b          	lock decq (%rbx)
   cd777:	75 0b                	jne    cd784 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x734>
   cd779:	48 8d 7c 24 38       	lea    0x38(%rsp),%rdi
   cd77e:	ff 15 ac 13 0a 00    	call   *0xa13ac(%rip)        # 16eb30 <_DYNAMIC+0x2a0>
   cd784:	48 8d 7c 24 68       	lea    0x68(%rsp),%rdi
   cd789:	e8 72 30 fe ff       	call   b0800 <core::ptr::drop_glue::<hyperreal::computable::node::Computable>>
   cd78e:	c7 44 24 04 00 00 00 	movl   $0x0,0x4(%rsp)
   cd795:	00 
   cd796:	48 8d 7c 24 40       	lea    0x40(%rsp),%rdi
   cd79b:	e8 00 f9 03 00       	call   10d0a0 <<alloc::vec::into_iter::IntoIter<(hyperreal::computable::node::Computable, hyperreal::rational::arithmetic::Rational)> as core::ops::drop::Drop>::drop>
   cd7a0:	31 db                	xor    %ebx,%ebx
   cd7a2:	e9 90 01 00 00       	jmp    cd937 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x8e7>
   cd7a7:	ff 15 8b 13 0a 00    	call   *0xa138b(%rip)        # 16eb38 <_DYNAMIC+0x2a8>
   cd7ad:	4c 89 6c 24 60       	mov    %r13,0x60(%rsp)
   cd7b2:	48 89 c5             	mov    %rax,%rbp
   cd7b5:	e9 b3 01 00 00       	jmp    cd96d <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x91d>
   cd7ba:	4d 89 ee             	mov    %r13,%r14
   cd7bd:	48 89 c5             	mov    %rax,%rbp
   cd7c0:	48 8b 33             	mov    (%rbx),%rsi
   cd7c3:	48 85 f6             	test   %rsi,%rsi
   cd7c6:	74 18                	je     cd7e0 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x790>
   cd7c8:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
   cd7cd:	48 8b 78 08          	mov    0x8(%rax),%rdi
   cd7d1:	48 c1 e6 04          	shl    $0x4,%rsi
   cd7d5:	ba 08 00 00 00       	mov    $0x8,%edx
   cd7da:	ff 15 70 13 0a 00    	call   *0xa1370(%rip)        # 16eb50 <_DYNAMIC+0x2c0>
   cd7e0:	4c 89 f7             	mov    %r14,%rdi
   cd7e3:	e8 b8 20 fe ff       	call   af8a0 <core::ptr::drop_glue::<alloc::vec::Vec<(hyperreal::computable::node::Computable, hyperreal::rational::arithmetic::Rational)>>>
   cd7e8:	e9 fe 01 00 00       	jmp    cd9eb <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x99b>
   cd7ed:	48 89 c5             	mov    %rax,%rbp
   cd7f0:	48 8d 7c 24 40       	lea    0x40(%rsp),%rdi
   cd7f5:	e8 06 30 fe ff       	call   b0800 <core::ptr::drop_glue::<hyperreal::computable::node::Computable>>
   cd7fa:	eb 03                	jmp    cd7ff <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x7af>
   cd7fc:	48 89 c5             	mov    %rax,%rbp
   cd7ff:	c7 44 24 04 00 00 00 	movl   $0x0,0x4(%rsp)
   cd806:	00 
   cd807:	31 db                	xor    %ebx,%ebx
   cd809:	e9 29 01 00 00       	jmp    cd937 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x8e7>
   cd80e:	48 89 c5             	mov    %rax,%rbp
   cd811:	f0 49 ff 0c 24       	lock decq (%r12)
   cd816:	41 b6 01             	mov    $0x1,%r14b
   cd819:	0f 85 9a 00 00 00    	jne    cd8b9 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x869>
   cd81f:	48 8d 7c 24 68       	lea    0x68(%rsp),%rdi
   cd824:	ff 15 06 13 0a 00    	call   *0xa1306(%rip)        # 16eb30 <_DYNAMIC+0x2a0>
   cd82a:	e9 8a 00 00 00       	jmp    cd8b9 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x869>
   cd82f:	48 89 c5             	mov    %rax,%rbp
   cd832:	eb 26                	jmp    cd85a <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x80a>
   cd834:	48 89 c5             	mov    %rax,%rbp
   cd837:	f0 48 ff 0b          	lock decq (%rbx)
   cd83b:	75 0b                	jne    cd848 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x7f8>
   cd83d:	48 8d 7c 24 68       	lea    0x68(%rsp),%rdi
   cd842:	ff 15 e8 12 0a 00    	call   *0xa12e8(%rip)        # 16eb30 <_DYNAMIC+0x2a0>
   cd848:	f0 49 ff 0c 24       	lock decq (%r12)
   cd84d:	75 0b                	jne    cd85a <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x80a>
   cd84f:	48 8d 7c 24 38       	lea    0x38(%rsp),%rdi
   cd854:	ff 15 d6 12 0a 00    	call   *0xa12d6(%rip)        # 16eb30 <_DYNAMIC+0x2a0>
   cd85a:	45 31 f6             	xor    %r14d,%r14d
   cd85d:	eb 5a                	jmp    cd8b9 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x869>
   cd85f:	ff 15 d3 12 0a 00    	call   *0xa12d3(%rip)        # 16eb38 <_DYNAMIC+0x2a8>
   cd865:	eb 00                	jmp    cd867 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x817>
   cd867:	48 89 c5             	mov    %rax,%rbp
   cd86a:	41 b6 01             	mov    $0x1,%r14b
   cd86d:	eb 4a                	jmp    cd8b9 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x869>
   cd86f:	48 89 c5             	mov    %rax,%rbp
   cd872:	f0 49 ff 0f          	lock decq (%r15)
   cd876:	b0 01                	mov    $0x1,%al
   cd878:	89 44 24 04          	mov    %eax,0x4(%rsp)
   cd87c:	0f 85 b3 00 00 00    	jne    cd935 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x8e5>
   cd882:	b0 01                	mov    $0x1,%al
   cd884:	89 44 24 04          	mov    %eax,0x4(%rsp)
   cd888:	48 8d 7c 24 40       	lea    0x40(%rsp),%rdi
   cd88d:	ff 15 9d 12 0a 00    	call   *0xa129d(%rip)        # 16eb30 <_DYNAMIC+0x2a0>
   cd893:	b3 01                	mov    $0x1,%bl
   cd895:	e9 9d 00 00 00       	jmp    cd937 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x8e7>
   cd89a:	e9 8d 00 00 00       	jmp    cd92c <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x8dc>
   cd89f:	48 89 c5             	mov    %rax,%rbp
   cd8a2:	f0 48 ff 0b          	lock decq (%rbx)
   cd8a6:	41 b6 01             	mov    $0x1,%r14b
   cd8a9:	75 0e                	jne    cd8b9 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x869>
   cd8ab:	48 8d bc 24 f0 00 00 	lea    0xf0(%rsp),%rdi
   cd8b2:	00 
   cd8b3:	ff 15 77 12 0a 00    	call   *0xa1277(%rip)        # 16eb30 <_DYNAMIC+0x2a0>
   cd8b9:	48 8b 44 24 78       	mov    0x78(%rsp),%rax
   cd8be:	f0 48 ff 08          	lock decq (%rax)
   cd8c2:	75 0b                	jne    cd8cf <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x87f>
   cd8c4:	48 8d 7c 24 28       	lea    0x28(%rsp),%rdi
   cd8c9:	ff 15 61 12 0a 00    	call   *0xa1261(%rip)        # 16eb30 <_DYNAMIC+0x2a0>
   cd8cf:	48 8b 44 24 30       	mov    0x30(%rsp),%rax
   cd8d4:	f0 48 ff 08          	lock decq (%rax)
   cd8d8:	75 0b                	jne    cd8e5 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x895>
   cd8da:	48 8d 7c 24 10       	lea    0x10(%rsp),%rdi
   cd8df:	ff 15 4b 12 0a 00    	call   *0xa124b(%rip)        # 16eb30 <_DYNAMIC+0x2a0>
   cd8e5:	48 8d 7c 24 40       	lea    0x40(%rsp),%rdi
   cd8ea:	e8 81 f8 03 00       	call   10d170 <<alloc::vec::into_iter::IntoIter<(hyperreal::rational::arithmetic::Rational, hyperreal::rational::arithmetic::Rational)> as core::ops::drop::Drop>::drop>
   cd8ef:	b0 01                	mov    $0x1,%al
   cd8f1:	89 44 24 04          	mov    %eax,0x4(%rsp)
   cd8f5:	31 db                	xor    %ebx,%ebx
   cd8f7:	45 84 f6             	test   %r14b,%r14b
   cd8fa:	75 3b                	jne    cd937 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x8e7>
   cd8fc:	eb 55                	jmp    cd953 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x903>
   cd8fe:	4c 89 6c 24 60       	mov    %r13,0x60(%rsp)
   cd903:	48 89 c5             	mov    %rax,%rbp
   cd906:	f0 49 ff 0f          	lock decq (%r15)
   cd90a:	b0 01                	mov    $0x1,%al
   cd90c:	89 44 24 04          	mov    %eax,0x4(%rsp)
   cd910:	75 23                	jne    cd935 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x8e5>
   cd912:	b0 01                	mov    $0x1,%al
   cd914:	89 44 24 04          	mov    %eax,0x4(%rsp)
   cd918:	48 8d 7c 24 40       	lea    0x40(%rsp),%rdi
   cd91d:	ff 15 0d 12 0a 00    	call   *0xa120d(%rip)        # 16eb30 <_DYNAMIC+0x2a0>
   cd923:	b3 01                	mov    $0x1,%bl
   cd925:	eb 10                	jmp    cd937 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x8e7>
   cd927:	4c 89 6c 24 60       	mov    %r13,0x60(%rsp)
   cd92c:	48 89 c5             	mov    %rax,%rbp
   cd92f:	b0 01                	mov    $0x1,%al
   cd931:	89 44 24 04          	mov    %eax,0x4(%rsp)
   cd935:	b3 01                	mov    $0x1,%bl
   cd937:	48 8b 84 24 b8 00 00 	mov    0xb8(%rsp),%rax
   cd93e:	00 
   cd93f:	f0 48 ff 08          	lock decq (%rax)
   cd943:	75 0e                	jne    cd953 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x903>
   cd945:	48 8d bc 24 b8 00 00 	lea    0xb8(%rsp),%rdi
   cd94c:	00 
   cd94d:	ff 15 dd 11 0a 00    	call   *0xa11dd(%rip)        # 16eb30 <_DYNAMIC+0x2a0>
   cd953:	48 8d bc 24 88 00 00 	lea    0x88(%rsp),%rdi
   cd95a:	00 
   cd95b:	e8 10 20 fe ff       	call   af970 <core::ptr::drop_glue::<alloc::vec::Vec<(hyperreal::rational::arithmetic::Rational, hyperreal::rational::arithmetic::Rational)>>>
   cd960:	48 8d bc 24 a0 00 00 	lea    0xa0(%rsp),%rdi
   cd967:	00 
   cd968:	e8 33 1f fe ff       	call   af8a0 <core::ptr::drop_glue::<alloc::vec::Vec<(hyperreal::computable::node::Computable, hyperreal::rational::arithmetic::Rational)>>>
   cd96d:	48 8b 44 24 20       	mov    0x20(%rsp),%rax
   cd972:	48 8b 00             	mov    (%rax),%rax
   cd975:	f0 48 ff 08          	lock decq (%rax)
   cd979:	75 0b                	jne    cd986 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x936>
   cd97b:	48 8b 7c 24 20       	mov    0x20(%rsp),%rdi
   cd980:	ff 15 aa 11 0a 00    	call   *0xa11aa(%rip)        # 16eb30 <_DYNAMIC+0x2a0>
   cd986:	84 db                	test   %bl,%bl
   cd988:	74 0a                	je     cd994 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x944>
   cd98a:	48 8b 7c 24 08       	mov    0x8(%rsp),%rdi
   cd98f:	e8 dc 1f fe ff       	call   af970 <core::ptr::drop_glue::<alloc::vec::Vec<(hyperreal::rational::arithmetic::Rational, hyperreal::rational::arithmetic::Rational)>>>
   cd994:	80 7c 24 04 00       	cmpb   $0x0,0x4(%rsp)
   cd999:	74 50                	je     cd9eb <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x99b>
   cd99b:	48 8b 7c 24 60       	mov    0x60(%rsp),%rdi
   cd9a0:	e8 fb 1e fe ff       	call   af8a0 <core::ptr::drop_glue::<alloc::vec::Vec<(hyperreal::computable::node::Computable, hyperreal::rational::arithmetic::Rational)>>>
   cd9a5:	eb 44                	jmp    cd9eb <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x99b>
   cd9a7:	ff 15 8b 11 0a 00    	call   *0xa118b(%rip)        # 16eb38 <_DYNAMIC+0x2a8>
   cd9ad:	48 89 c5             	mov    %rax,%rbp
   cd9b0:	49 ff ce             	dec    %r14
   cd9b3:	74 11                	je     cd9c6 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x976>
   cd9b5:	4c 8d 7b 18          	lea    0x18(%rbx),%r15
   cd9b9:	48 89 df             	mov    %rbx,%rdi
   cd9bc:	e8 af 36 04 00       	call   111070 <core::ptr::drop_glue::<(hyperreal::computable::node::Computable, hyperreal::rational::arithmetic::Rational)>>
   cd9c1:	4c 89 fb             	mov    %r15,%rbx
   cd9c4:	eb ea                	jmp    cd9b0 <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x960>
   cd9c6:	49 8b 45 00          	mov    0x0(%r13),%rax
   cd9ca:	48 85 c0             	test   %rax,%rax
   cd9cd:	74 1c                	je     cd9eb <<hyperreal::computable::node::RadicalCoefficient>::multiply+0x99b>
   cd9cf:	48 8b 4c 24 08       	mov    0x8(%rsp),%rcx
   cd9d4:	48 8b 79 20          	mov    0x20(%rcx),%rdi
   cd9d8:	48 c1 e0 03          	shl    $0x3,%rax
   cd9dc:	48 8d 34 40          	lea    (%rax,%rax,2),%rsi
   cd9e0:	ba 08 00 00 00       	mov    $0x8,%edx
   cd9e5:	ff 15 65 11 0a 00    	call   *0xa1165(%rip)        # 16eb50 <_DYNAMIC+0x2c0>
   cd9eb:	48 89 ef             	mov    %rbp,%rdi
   cd9ee:	e8 3d ae 09 00       	call   168830 <_Unwind_Resume@plt>
   cd9f3:	ff 15 3f 11 0a 00    	call   *0xa113f(%rip)        # 16eb38 <_DYNAMIC+0x2a8>
