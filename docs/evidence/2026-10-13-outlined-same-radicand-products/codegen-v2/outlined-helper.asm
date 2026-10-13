
/home/tim/Documents/GitHub/workspace/csgrs/target/outlined-same-radicand-products/cost-v2-binaries/outlined-quadratic-time:     file format elf64-x86-64


Disassembly of section .text:

00000000000ccde0 <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand>:
   ccde0:	55                   	push   %rbp
   ccde1:	41 57                	push   %r15
   ccde3:	41 56                	push   %r14
   ccde5:	41 55                	push   %r13
   ccde7:	41 54                	push   %r12
   ccde9:	53                   	push   %rbx
   ccdea:	48 83 ec 68          	sub    $0x68,%rsp
   ccdee:	48 83 7e 10 00       	cmpq   $0x0,0x10(%rsi)
   ccdf3:	48 89 74 24 10       	mov    %rsi,0x10(%rsp)
   ccdf8:	0f 84 07 03 00 00    	je     cd105 <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x325>
   ccdfe:	49 89 d7             	mov    %rdx,%r15
   cce01:	48 83 7a 10 00       	cmpq   $0x0,0x10(%rdx)
   cce06:	0f 84 02 03 00 00    	je     cd10e <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x32e>
   cce0c:	49 89 fd             	mov    %rdi,%r13
   cce0f:	4c 8b 76 08          	mov    0x8(%rsi),%r14
   cce13:	4d 8b 67 08          	mov    0x8(%r15),%r12
   cce17:	48 8d 7e 30          	lea    0x30(%rsi),%rdi
   cce1b:	49 83 c7 30          	add    $0x30,%r15
   cce1f:	4c 89 fe             	mov    %r15,%rsi
   cce22:	e8 d9 49 fc ff       	call   91800 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cce27:	49 8d 7e 08          	lea    0x8(%r14),%rdi
   cce2b:	48 89 44 24 38       	mov    %rax,0x38(%rsp)
   cce30:	48 89 44 24 50       	mov    %rax,0x50(%rsp)
   cce35:	4c 89 fe             	mov    %r15,%rsi
   cce38:	e8 c3 49 fc ff       	call   91800 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cce3d:	49 89 c7             	mov    %rax,%r15
   cce40:	4c 89 64 24 48       	mov    %r12,0x48(%rsp)
   cce45:	49 8d 7c 24 08       	lea    0x8(%r12),%rdi
   cce4a:	48 89 44 24 30       	mov    %rax,0x30(%rsp)
   cce4f:	41 b4 01             	mov    $0x1,%r12b
   cce52:	40 b5 01             	mov    $0x1,%bpl
   cce55:	48 89 fb             	mov    %rdi,%rbx
   cce58:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
   cce5d:	48 8d 70 30          	lea    0x30(%rax),%rsi
   cce61:	e8 9a 49 fc ff       	call   91800 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   cce66:	48 89 44 24 40       	mov    %rax,0x40(%rsp)
   cce6b:	48 89 c6             	mov    %rax,%rsi
   cce6e:	4c 89 f8             	mov    %r15,%rax
   cce71:	45 0f b6 7f 64       	movzbl 0x64(%r15),%r15d
   cce76:	41 80 ff 01          	cmp    $0x1,%r15b
   cce7a:	48 89 74 24 20       	mov    %rsi,0x20(%rsp)
   cce7f:	75 1b                	jne    cce9c <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0xbc>
   cce81:	f0 48 ff 08          	lock decq (%rax)
   cce85:	48 89 f0             	mov    %rsi,%rax
   cce88:	75 39                	jne    ccec3 <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0xe3>
   cce8a:	48 8d 7c 24 30       	lea    0x30(%rsp),%rdi
   cce8f:	ff 15 4b 25 0a 00    	call   *0xa254b(%rip)        # 16f3e0 <_DYNAMIC+0x2a0>
   cce95:	48 8b 44 24 20       	mov    0x20(%rsp),%rax
   cce9a:	eb 27                	jmp    ccec3 <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0xe3>
   cce9c:	48 89 44 24 18       	mov    %rax,0x18(%rsp)
   ccea1:	80 7e 64 01          	cmpb   $0x1,0x64(%rsi)
   ccea5:	75 08                	jne    cceaf <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0xcf>
   ccea7:	b0 01                	mov    $0x1,%al
   ccea9:	89 44 24 0c          	mov    %eax,0xc(%rsp)
   ccead:	eb 26                	jmp    cced5 <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0xf5>
   cceaf:	31 ed                	xor    %ebp,%ebp
   cceb1:	c7 44 24 0c 00 00 00 	movl   $0x0,0xc(%rsp)
   cceb8:	00 
   cceb9:	48 8b 7c 24 18       	mov    0x18(%rsp),%rdi
   ccebe:	e8 0d 30 fc ff       	call   8fed0 <<hyperreal::rational::arithmetic::Rational as core::ops::arith::Add>::add>
   ccec3:	48 89 44 24 18       	mov    %rax,0x18(%rsp)
   ccec8:	48 89 44 24 30       	mov    %rax,0x30(%rsp)
   ccecd:	c7 44 24 0c 00 00 00 	movl   $0x0,0xc(%rsp)
   cced4:	00 
   cced5:	40 b5 01             	mov    $0x1,%bpl
   cced8:	49 8d 7e 08          	lea    0x8(%r14),%rdi
   ccedc:	48 89 de             	mov    %rbx,%rsi
   ccedf:	e8 1c 49 fc ff       	call   91800 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   ccee4:	49 89 c4             	mov    %rax,%r12
   ccee7:	48 8b 44 24 38       	mov    0x38(%rsp),%rax
   cceec:	48 89 44 24 60       	mov    %rax,0x60(%rsp)
   ccef1:	4c 89 64 24 28       	mov    %r12,0x28(%rsp)
   ccef6:	48 8d 7c 24 28       	lea    0x28(%rsp),%rdi
   ccefb:	4c 89 f6             	mov    %r14,%rsi
   ccefe:	e8 fd 48 fc ff       	call   91800 <<&hyperreal::rational::arithmetic::Rational as core::ops::arith::Mul>::mul>
   ccf03:	49 89 c6             	mov    %rax,%r14
   ccf06:	f0 49 ff 0c 24       	lock decq (%r12)
   ccf0b:	48 8b 5c 24 10       	mov    0x10(%rsp),%rbx
   ccf10:	48 8d 6b 30          	lea    0x30(%rbx),%rbp
   ccf14:	75 0b                	jne    ccf21 <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x141>
   ccf16:	48 8d 7c 24 28       	lea    0x28(%rsp),%rdi
   ccf1b:	ff 15 bf 24 0a 00    	call   *0xa24bf(%rip)        # 16f3e0 <_DYNAMIC+0x2a0>
   ccf21:	48 8b 7c 24 38       	mov    0x38(%rsp),%rdi
   ccf26:	4c 89 f6             	mov    %r14,%rsi
   ccf29:	e8 a2 2f fc ff       	call   8fed0 <<hyperreal::rational::arithmetic::Rational as core::ops::arith::Add>::add>
   ccf2e:	49 89 c6             	mov    %rax,%r14
   ccf31:	48 89 44 24 58       	mov    %rax,0x58(%rsp)
   ccf36:	48 8b 44 24 18       	mov    0x18(%rsp),%rax
   ccf3b:	80 78 64 01          	cmpb   $0x1,0x64(%rax)
   ccf3f:	75 74                	jne    ccfb5 <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x1d5>
   ccf41:	4d 89 75 30          	mov    %r14,0x30(%r13)
   ccf45:	49 c7 45 00 00 00 00 	movq   $0x0,0x0(%r13)
   ccf4c:	00 
   ccf4d:	49 c7 45 08 08 00 00 	movq   $0x8,0x8(%r13)
   ccf54:	00 
   ccf55:	0f 57 c0             	xorps  %xmm0,%xmm0
   ccf58:	41 0f 11 45 10       	movups %xmm0,0x10(%r13)
   ccf5d:	49 c7 45 20 08 00 00 	movq   $0x8,0x20(%r13)
   ccf64:	00 
   ccf65:	49 c7 45 28 00 00 00 	movq   $0x0,0x28(%r13)
   ccf6c:	00 
   ccf6d:	80 7c 24 0c 00       	cmpb   $0x0,0xc(%rsp)
   ccf72:	4c 8b 7c 24 18       	mov    0x18(%rsp),%r15
   ccf77:	48 8b 44 24 20       	mov    0x20(%rsp),%rax
   ccf7c:	74 17                	je     ccf95 <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x1b5>
   ccf7e:	f0 48 ff 08          	lock decq (%rax)
   ccf82:	75 11                	jne    ccf95 <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x1b5>
   ccf84:	40 b5 01             	mov    $0x1,%bpl
   ccf87:	45 31 e4             	xor    %r12d,%r12d
   ccf8a:	48 8d 7c 24 40       	lea    0x40(%rsp),%rdi
   ccf8f:	ff 15 4b 24 0a 00    	call   *0xa244b(%rip)        # 16f3e0 <_DYNAMIC+0x2a0>
   ccf95:	f0 49 ff 0f          	lock decq (%r15)
   ccf99:	75 0b                	jne    ccfa6 <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x1c6>
   ccf9b:	48 8d 7c 24 30       	lea    0x30(%rsp),%rdi
   ccfa0:	ff 15 3a 24 0a 00    	call   *0xa243a(%rip)        # 16f3e0 <_DYNAMIC+0x2a0>
   ccfa6:	48 8b 7c 24 10       	mov    0x10(%rsp),%rdi
   ccfab:	e8 50 3e fe ff       	call   b0e00 <core::ptr::drop_glue::<hyperreal::computable::node::RadicalCoefficient>>
   ccfb0:	e9 41 01 00 00       	jmp    cd0f6 <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x316>
   ccfb5:	41 80 ff 01          	cmp    $0x1,%r15b
   ccfb9:	75 40                	jne    ccffb <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x21b>
   ccfbb:	48 8b 4c 24 48       	mov    0x48(%rsp),%rcx
   ccfc0:	48 8b 01             	mov    (%rcx),%rax
   ccfc3:	f0 48 ff 00          	lock incq (%rax)
   ccfc7:	0f 8e 78 01 00 00    	jle    cd145 <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x365>
   ccfcd:	4c 8b 39             	mov    (%rcx),%r15
   ccfd0:	4c 89 7c 24 28       	mov    %r15,0x28(%rsp)
   ccfd5:	48 83 7b 10 00       	cmpq   $0x0,0x10(%rbx)
   ccfda:	0f 84 54 01 00 00    	je     cd134 <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x354>
   ccfe0:	4c 8b 63 08          	mov    0x8(%rbx),%r12
   ccfe4:	49 8b 04 24          	mov    (%r12),%rax
   ccfe8:	f0 48 ff 08          	lock decq (%rax)
   ccfec:	75 09                	jne    ccff7 <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x217>
   ccfee:	4c 89 e7             	mov    %r12,%rdi
   ccff1:	ff 15 e9 23 0a 00    	call   *0xa23e9(%rip)        # 16f3e0 <_DYNAMIC+0x2a0>
   ccff7:	4d 89 3c 24          	mov    %r15,(%r12)
   ccffb:	48 8b 45 00          	mov    0x0(%rbp),%rax
   ccfff:	f0 48 ff 08          	lock decq (%rax)
   cd003:	4c 8b 7c 24 18       	mov    0x18(%rsp),%r15
   cd008:	75 09                	jne    cd013 <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x233>
   cd00a:	48 89 ef             	mov    %rbp,%rdi
   cd00d:	ff 15 cd 23 0a 00    	call   *0xa23cd(%rip)        # 16f3e0 <_DYNAMIC+0x2a0>
   cd013:	4c 89 73 30          	mov    %r14,0x30(%rbx)
   cd017:	4c 89 7c 24 28       	mov    %r15,0x28(%rsp)
   cd01c:	48 83 7b 10 00       	cmpq   $0x0,0x10(%rbx)
   cd021:	0f 84 fa 00 00 00    	je     cd121 <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x341>
   cd027:	4c 8b 73 08          	mov    0x8(%rbx),%r14
   cd02b:	49 8b 46 08          	mov    0x8(%r14),%rax
   cd02f:	49 83 c6 08          	add    $0x8,%r14
   cd033:	f0 48 ff 08          	lock decq (%rax)
   cd037:	75 09                	jne    cd042 <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x262>
   cd039:	4c 89 f7             	mov    %r14,%rdi
   cd03c:	ff 15 9e 23 0a 00    	call   *0xa239e(%rip)        # 16f3e0 <_DYNAMIC+0x2a0>
   cd042:	4d 89 3e             	mov    %r15,(%r14)
   cd045:	48 8b 7b 20          	mov    0x20(%rbx),%rdi
   cd049:	4c 8b 7b 28          	mov    0x28(%rbx),%r15
   cd04d:	49 ff c7             	inc    %r15
   cd050:	49 83 ff 01          	cmp    $0x1,%r15
   cd054:	74 11                	je     cd067 <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x287>
   cd056:	4c 8d 77 18          	lea    0x18(%rdi),%r14
   cd05a:	49 ff cf             	dec    %r15
   cd05d:	e8 5e 48 04 00       	call   1118c0 <core::ptr::drop_glue::<(hyperreal::computable::node::Computable, hyperreal::rational::arithmetic::Rational)>>
   cd062:	4c 89 f7             	mov    %r14,%rdi
   cd065:	eb e9                	jmp    cd050 <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x270>
   cd067:	48 8b 43 18          	mov    0x18(%rbx),%rax
   cd06b:	48 85 c0             	test   %rax,%rax
   cd06e:	74 17                	je     cd087 <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x2a7>
   cd070:	48 8b 7b 20          	mov    0x20(%rbx),%rdi
   cd074:	48 c1 e0 03          	shl    $0x3,%rax
   cd078:	48 8d 34 40          	lea    (%rax,%rax,2),%rsi
   cd07c:	ba 08 00 00 00       	mov    $0x8,%edx
   cd081:	ff 15 79 23 0a 00    	call   *0xa2379(%rip)        # 16f400 <_DYNAMIC+0x2c0>
   cd087:	48 c7 43 18 00 00 00 	movq   $0x0,0x18(%rbx)
   cd08e:	00 
   cd08f:	48 c7 43 20 08 00 00 	movq   $0x8,0x20(%rbx)
   cd096:	00 
   cd097:	48 c7 43 28 00 00 00 	movq   $0x0,0x28(%rbx)
   cd09e:	00 
   cd09f:	48 8b 43 30          	mov    0x30(%rbx),%rax
   cd0a3:	49 89 45 30          	mov    %rax,0x30(%r13)
   cd0a7:	0f 10 03             	movups (%rbx),%xmm0
   cd0aa:	41 0f 11 45 00       	movups %xmm0,0x0(%r13)
   cd0af:	48 8b 43 20          	mov    0x20(%rbx),%rax
   cd0b3:	49 89 45 20          	mov    %rax,0x20(%r13)
   cd0b7:	48 8b 43 28          	mov    0x28(%rbx),%rax
   cd0bb:	49 89 45 28          	mov    %rax,0x28(%r13)
   cd0bf:	48 8b 43 10          	mov    0x10(%rbx),%rax
   cd0c3:	49 89 45 10          	mov    %rax,0x10(%r13)
   cd0c7:	48 8b 43 18          	mov    0x18(%rbx),%rax
   cd0cb:	49 89 45 18          	mov    %rax,0x18(%r13)
   cd0cf:	80 7c 24 0c 00       	cmpb   $0x0,0xc(%rsp)
   cd0d4:	4c 8b 7c 24 18       	mov    0x18(%rsp),%r15
   cd0d9:	48 8b 44 24 20       	mov    0x20(%rsp),%rax
   cd0de:	74 16                	je     cd0f6 <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x316>
   cd0e0:	f0 48 ff 08          	lock decq (%rax)
   cd0e4:	75 10                	jne    cd0f6 <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x316>
   cd0e6:	45 31 e4             	xor    %r12d,%r12d
   cd0e9:	48 8d 7c 24 40       	lea    0x40(%rsp),%rdi
   cd0ee:	31 ed                	xor    %ebp,%ebp
   cd0f0:	ff 15 ea 22 0a 00    	call   *0xa22ea(%rip)        # 16f3e0 <_DYNAMIC+0x2a0>
   cd0f6:	48 83 c4 68          	add    $0x68,%rsp
   cd0fa:	5b                   	pop    %rbx
   cd0fb:	41 5c                	pop    %r12
   cd0fd:	41 5d                	pop    %r13
   cd0ff:	41 5e                	pop    %r14
   cd101:	41 5f                	pop    %r15
   cd103:	5d                   	pop    %rbp
   cd104:	c3                   	ret
   cd105:	48 8d 15 64 e6 09 00 	lea    0x9e664(%rip),%rdx        # 16b770 <anon.afeede234644f26776946e76b404b106.261.llvm.3222190057612512104+0x128>
   cd10c:	eb 07                	jmp    cd115 <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x335>
   cd10e:	48 8d 15 73 e6 09 00 	lea    0x9e673(%rip),%rdx        # 16b788 <anon.afeede234644f26776946e76b404b106.261.llvm.3222190057612512104+0x140>
   cd115:	31 ff                	xor    %edi,%edi
   cd117:	31 f6                	xor    %esi,%esi
   cd119:	ff 15 a1 27 0a 00    	call   *0xa27a1(%rip)        # 16f8c0 <_DYNAMIC+0x780>
   cd11f:	eb 24                	jmp    cd145 <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x365>
   cd121:	48 8d 15 90 e6 09 00 	lea    0x9e690(%rip),%rdx        # 16b7b8 <anon.afeede234644f26776946e76b404b106.261.llvm.3222190057612512104+0x170>
   cd128:	31 ff                	xor    %edi,%edi
   cd12a:	31 f6                	xor    %esi,%esi
   cd12c:	ff 15 8e 27 0a 00    	call   *0xa278e(%rip)        # 16f8c0 <_DYNAMIC+0x780>
   cd132:	eb 11                	jmp    cd145 <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x365>
   cd134:	48 8d 15 65 e6 09 00 	lea    0x9e665(%rip),%rdx        # 16b7a0 <anon.afeede234644f26776946e76b404b106.261.llvm.3222190057612512104+0x158>
   cd13b:	31 ff                	xor    %edi,%edi
   cd13d:	31 f6                	xor    %esi,%esi
   cd13f:	ff 15 7b 27 0a 00    	call   *0xa277b(%rip)        # 16f8c0 <_DYNAMIC+0x780>
   cd145:	0f 0b                	ud2
   cd147:	49 89 c5             	mov    %rax,%r13
   cd14a:	4d 89 3c 24          	mov    %r15,(%r12)
   cd14e:	eb 2b                	jmp    cd17b <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x39b>
   cd150:	49 89 c5             	mov    %rax,%r13
   cd153:	4d 89 3e             	mov    %r15,(%r14)
   cd156:	e9 72 01 00 00       	jmp    cd2cd <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x4ed>
   cd15b:	49 89 c5             	mov    %rax,%r13
   cd15e:	4c 89 75 00          	mov    %r14,0x0(%rbp)
   cd162:	e9 87 00 00 00       	jmp    cd1ee <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x40e>
   cd167:	49 89 c5             	mov    %rax,%r13
   cd16a:	f0 49 ff 0f          	lock decq (%r15)
   cd16e:	75 0b                	jne    cd17b <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x39b>
   cd170:	48 8d 7c 24 28       	lea    0x28(%rsp),%rdi
   cd175:	ff 15 65 22 0a 00    	call   *0xa2265(%rip)        # 16f3e0 <_DYNAMIC+0x2a0>
   cd17b:	f0 49 ff 0e          	lock decq (%r14)
   cd17f:	41 b7 01             	mov    $0x1,%r15b
   cd182:	0f 85 9f 00 00 00    	jne    cd227 <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x447>
   cd188:	48 8d 7c 24 58       	lea    0x58(%rsp),%rdi
   cd18d:	ff 15 4d 22 0a 00    	call   *0xa224d(%rip)        # 16f3e0 <_DYNAMIC+0x2a0>
   cd193:	e9 8f 00 00 00       	jmp    cd227 <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x447>
   cd198:	e9 be 00 00 00       	jmp    cd25b <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x47b>
   cd19d:	49 89 c5             	mov    %rax,%r13
   cd1a0:	48 8b 44 24 20       	mov    0x20(%rsp),%rax
   cd1a5:	48 89 44 24 30       	mov    %rax,0x30(%rsp)
   cd1aa:	40 b5 01             	mov    $0x1,%bpl
   cd1ad:	41 b4 01             	mov    $0x1,%r12b
   cd1b0:	e9 4d 01 00 00       	jmp    cd302 <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x522>
   cd1b5:	49 89 c5             	mov    %rax,%r13
   cd1b8:	45 31 ff             	xor    %r15d,%r15d
   cd1bb:	48 8b 44 24 18       	mov    0x18(%rsp),%rax
   cd1c0:	f0 48 ff 08          	lock decq (%rax)
   cd1c4:	41 bc 00 00 00 00    	mov    $0x0,%r12d
   cd1ca:	0f 85 03 01 00 00    	jne    cd2d3 <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x4f3>
   cd1d0:	45 31 e4             	xor    %r12d,%r12d
   cd1d3:	48 8d 7c 24 28       	lea    0x28(%rsp),%rdi
   cd1d8:	ff 15 02 22 0a 00    	call   *0xa2202(%rip)        # 16f3e0 <_DYNAMIC+0x2a0>
   cd1de:	45 31 ff             	xor    %r15d,%r15d
   cd1e1:	e9 ed 00 00 00       	jmp    cd2d3 <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x4f3>
   cd1e6:	49 89 c5             	mov    %rax,%r13
   cd1e9:	eb 23                	jmp    cd20e <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x42e>
   cd1eb:	49 89 c5             	mov    %rax,%r13
   cd1ee:	41 b7 01             	mov    $0x1,%r15b
   cd1f1:	45 31 e4             	xor    %r12d,%r12d
   cd1f4:	e9 da 00 00 00       	jmp    cd2d3 <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x4f3>
   cd1f9:	49 89 c5             	mov    %rax,%r13
   cd1fc:	f0 49 ff 0c 24       	lock decq (%r12)
   cd201:	75 0b                	jne    cd20e <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x42e>
   cd203:	48 8d 7c 24 28       	lea    0x28(%rsp),%rdi
   cd208:	ff 15 d2 21 0a 00    	call   *0xa21d2(%rip)        # 16f3e0 <_DYNAMIC+0x2a0>
   cd20e:	48 8b 44 24 38       	mov    0x38(%rsp),%rax
   cd213:	f0 48 ff 08          	lock decq (%rax)
   cd217:	41 b7 01             	mov    $0x1,%r15b
   cd21a:	75 0b                	jne    cd227 <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x447>
   cd21c:	48 8d 7c 24 60       	lea    0x60(%rsp),%rdi
   cd221:	ff 15 b9 21 0a 00    	call   *0xa21b9(%rip)        # 16f3e0 <_DYNAMIC+0x2a0>
   cd227:	45 31 e4             	xor    %r12d,%r12d
   cd22a:	e9 a4 00 00 00       	jmp    cd2d3 <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x4f3>
   cd22f:	ff 15 b3 21 0a 00    	call   *0xa21b3(%rip)        # 16f3e8 <_DYNAMIC+0x2a8>
   cd235:	49 89 c5             	mov    %rax,%r13
   cd238:	40 b5 01             	mov    $0x1,%bpl
   cd23b:	e9 dd 00 00 00       	jmp    cd31d <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x53d>
   cd240:	4c 89 7c 24 18       	mov    %r15,0x18(%rsp)
   cd245:	49 89 c5             	mov    %rax,%r13
   cd248:	41 89 ef             	mov    %ebp,%r15d
   cd24b:	e9 a3 00 00 00       	jmp    cd2f3 <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x513>
   cd250:	41 89 ef             	mov    %ebp,%r15d
   cd253:	49 89 c5             	mov    %rax,%r13
   cd256:	41 b4 01             	mov    $0x1,%r12b
   cd259:	eb 78                	jmp    cd2d3 <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x4f3>
   cd25b:	49 89 c5             	mov    %rax,%r13
   cd25e:	e9 d5 00 00 00       	jmp    cd338 <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x558>
   cd263:	49 89 c5             	mov    %rax,%r13
   cd266:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
   cd26d:	00 00 00 
   cd270:	49 ff cf             	dec    %r15
   cd273:	74 11                	je     cd286 <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x4a6>
   cd275:	4d 8d 66 18          	lea    0x18(%r14),%r12
   cd279:	4c 89 f7             	mov    %r14,%rdi
   cd27c:	e8 3f 46 04 00       	call   1118c0 <core::ptr::drop_glue::<(hyperreal::computable::node::Computable, hyperreal::rational::arithmetic::Rational)>>
   cd281:	4d 89 e6             	mov    %r12,%r14
   cd284:	eb ea                	jmp    cd270 <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x490>
   cd286:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
   cd28b:	48 8b 40 18          	mov    0x18(%rax),%rax
   cd28f:	48 85 c0             	test   %rax,%rax
   cd292:	74 1c                	je     cd2b0 <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x4d0>
   cd294:	48 8b 4c 24 10       	mov    0x10(%rsp),%rcx
   cd299:	48 8b 79 20          	mov    0x20(%rcx),%rdi
   cd29d:	48 c1 e0 03          	shl    $0x3,%rax
   cd2a1:	48 8d 34 40          	lea    (%rax,%rax,2),%rsi
   cd2a5:	ba 08 00 00 00       	mov    $0x8,%edx
   cd2aa:	ff 15 50 21 0a 00    	call   *0xa2150(%rip)        # 16f400 <_DYNAMIC+0x2c0>
   cd2b0:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
   cd2b5:	48 c7 40 18 00 00 00 	movq   $0x0,0x18(%rax)
   cd2bc:	00 
   cd2bd:	48 c7 40 20 08 00 00 	movq   $0x8,0x20(%rax)
   cd2c4:	00 
   cd2c5:	48 c7 40 28 00 00 00 	movq   $0x0,0x28(%rax)
   cd2cc:	00 
   cd2cd:	45 31 e4             	xor    %r12d,%r12d
   cd2d0:	45 31 ff             	xor    %r15d,%r15d
   cd2d3:	40 b5 01             	mov    $0x1,%bpl
   cd2d6:	80 7c 24 0c 00       	cmpb   $0x0,0xc(%rsp)
   cd2db:	74 16                	je     cd2f3 <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x513>
   cd2dd:	48 8b 44 24 20       	mov    0x20(%rsp),%rax
   cd2e2:	f0 48 ff 08          	lock decq (%rax)
   cd2e6:	75 0b                	jne    cd2f3 <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x513>
   cd2e8:	48 8d 7c 24 40       	lea    0x40(%rsp),%rdi
   cd2ed:	ff 15 ed 20 0a 00    	call   *0xa20ed(%rip)        # 16f3e0 <_DYNAMIC+0x2a0>
   cd2f3:	45 84 ff             	test   %r15b,%r15b
   cd2f6:	74 20                	je     cd318 <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x538>
   cd2f8:	48 8b 44 24 18       	mov    0x18(%rsp),%rax
   cd2fd:	48 89 44 24 20       	mov    %rax,0x20(%rsp)
   cd302:	48 8b 44 24 20       	mov    0x20(%rsp),%rax
   cd307:	f0 48 ff 08          	lock decq (%rax)
   cd30b:	75 0b                	jne    cd318 <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x538>
   cd30d:	48 8d 7c 24 30       	lea    0x30(%rsp),%rdi
   cd312:	ff 15 c8 20 0a 00    	call   *0xa20c8(%rip)        # 16f3e0 <_DYNAMIC+0x2a0>
   cd318:	45 84 e4             	test   %r12b,%r12b
   cd31b:	74 16                	je     cd333 <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x553>
   cd31d:	48 8b 44 24 38       	mov    0x38(%rsp),%rax
   cd322:	f0 48 ff 08          	lock decq (%rax)
   cd326:	75 0b                	jne    cd333 <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x553>
   cd328:	48 8d 7c 24 50       	lea    0x50(%rsp),%rdi
   cd32d:	ff 15 ad 20 0a 00    	call   *0xa20ad(%rip)        # 16f3e0 <_DYNAMIC+0x2a0>
   cd333:	40 84 ed             	test   %bpl,%bpl
   cd336:	74 36                	je     cd36e <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x58e>
   cd338:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
   cd33d:	48 8b 40 30          	mov    0x30(%rax),%rax
   cd341:	f0 48 ff 08          	lock decq (%rax)
   cd345:	75 0f                	jne    cd356 <<hyperreal::computable::node::RadicalCoefficient>::multiply_same_radicand+0x576>
   cd347:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
   cd34c:	48 8d 78 30          	lea    0x30(%rax),%rdi
   cd350:	ff 15 8a 20 0a 00    	call   *0xa208a(%rip)        # 16f3e0 <_DYNAMIC+0x2a0>
   cd356:	48 8b 7c 24 10       	mov    0x10(%rsp),%rdi
   cd35b:	e8 70 27 fe ff       	call   afad0 <core::ptr::drop_glue::<alloc::vec::Vec<(hyperreal::rational::arithmetic::Rational, hyperreal::rational::arithmetic::Rational)>>>
   cd360:	48 8b 7c 24 10       	mov    0x10(%rsp),%rdi
   cd365:	48 83 c7 18          	add    $0x18,%rdi
   cd369:	e8 92 26 fe ff       	call   afa00 <core::ptr::drop_glue::<alloc::vec::Vec<(hyperreal::computable::node::Computable, hyperreal::rational::arithmetic::Rational)>>>
   cd36e:	4c 89 ef             	mov    %r13,%rdi
   cd371:	e8 0a bd 09 00       	call   169080 <_Unwind_Resume@plt>
   cd376:	ff 15 6c 20 0a 00    	call   *0xa206c(%rip)        # 16f3e8 <_DYNAMIC+0x2a8>
   cd37c:	ff 15 66 20 0a 00    	call   *0xa2066(%rip)        # 16f3e8 <_DYNAMIC+0x2a8>
