
/home/tim/Documents/GitHub/workspace/csgrs/target/outlined-same-radicand-products/cost-v2-binaries/direct-quadratic-time:     file format elf64-x86-64


Disassembly of section .text:

00000000000cf680 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply>:
   cf680:	55                   	push   %rbp
   cf681:	41 57                	push   %r15
   cf683:	41 56                	push   %r14
   cf685:	41 55                	push   %r13
   cf687:	41 54                	push   %r12
   cf689:	53                   	push   %rbx
   cf68a:	48 81 ec d8 01 00 00 	sub    $0x1d8,%rsp
   cf691:	49 89 fe             	mov    %rdi,%r14
   cf694:	48 c7 44 24 40 00 00 	movq   $0x0,0x40(%rsp)
   cf69b:	00 00 
   cf69d:	48 c7 44 24 48 08 00 	movq   $0x8,0x48(%rsp)
   cf6a4:	00 00 
   cf6a6:	48 c7 44 24 50 00 00 	movq   $0x0,0x50(%rsp)
   cf6ad:	00 00 
   cf6af:	48 8b 42 08          	mov    0x8(%rdx),%rax
   cf6b3:	48 89 04 24          	mov    %rax,(%rsp)
   cf6b7:	48 89 54 24 58       	mov    %rdx,0x58(%rsp)
   cf6bc:	4c 8b 62 10          	mov    0x10(%rdx),%r12
   cf6c0:	49 83 fc 01          	cmp    $0x1,%r12
   cf6c4:	0f 85 b8 01 00 00    	jne    cf882 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x202>
   cf6ca:	48 8b 06             	mov    (%rsi),%rax
   cf6cd:	48 8b 5e 08          	mov    0x8(%rsi),%rbx
   cf6d1:	48 8b 4e 10          	mov    0x10(%rsi),%rcx
   cf6d5:	49 89 cd             	mov    %rcx,%r13
   cf6d8:	49 c1 e5 06          	shl    $0x6,%r13
   cf6dc:	4a 8d 14 2b          	lea    (%rbx,%r13,1),%rdx
   cf6e0:	48 89 5c 24 18       	mov    %rbx,0x18(%rsp)
   cf6e5:	48 89 5c 24 20       	mov    %rbx,0x20(%rsp)
   cf6ea:	48 89 44 24 28       	mov    %rax,0x28(%rsp)
   cf6ef:	48 89 54 24 30       	mov    %rdx,0x30(%rsp)
   cf6f4:	48 85 c9             	test   %rcx,%rcx
   cf6f7:	0f 84 64 01 00 00    	je     cf861 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x1e1>
   cf6fd:	48 89 54 24 60       	mov    %rdx,0x60(%rsp)
   cf702:	4c 89 74 24 10       	mov    %r14,0x10(%rsp)
   cf707:	48 8b 04 24          	mov    (%rsp),%rax
   cf70b:	48 83 c0 08          	add    $0x8,%rax
   cf70f:	48 89 44 24 38       	mov    %rax,0x38(%rsp)
   cf714:	4c 8d bc 24 98 00 00 	lea    0x98(%rsp),%r15
   cf71b:	00 
   cf71c:	48 83 c3 40          	add    $0x40,%rbx
   cf720:	48 8d ac 24 00 01 00 	lea    0x100(%rsp),%rbp
   cf727:	00 
   cf728:	0f 1f 84 00 00 00 00 	nopl   0x0(%rax,%rax,1)
   cf72f:	00 
   cf730:	48 8b 43 c8          	mov    -0x38(%rbx),%rax
   cf734:	48 83 f8 ff          	cmp    $0xffffffffffffffff,%rax
   cf738:	0f 84 19 01 00 00    	je     cf857 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x1d7>
   cf73e:	44 0f b7 73 c0       	movzwl -0x40(%rbx),%r14d
   cf743:	48 89 84 24 90 00 00 	mov    %rax,0x90(%rsp)
   cf74a:	00 
   cf74b:	0f 10 43 d0          	movups -0x30(%rbx),%xmm0
   cf74f:	0f 10 4b e0          	movups -0x20(%rbx),%xmm1
   cf753:	0f 10 53 f0          	movups -0x10(%rbx),%xmm2
   cf757:	41 0f 11 57 20       	movups %xmm2,0x20(%r15)
   cf75c:	41 0f 11 4f 10       	movups %xmm1,0x10(%r15)
   cf761:	41 0f 11 07          	movups %xmm0,(%r15)
   cf765:	48 8b 04 24          	mov    (%rsp),%rax
   cf769:	66 44 03 30          	add    (%rax),%r14w
   cf76d:	0f 80 7b 06 00 00    	jo     cfdee <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x76e>
   cf773:	48 8b 84 24 c0 00 00 	mov    0xc0(%rsp),%rax
   cf77a:	00 
   cf77b:	48 89 84 24 30 01 00 	mov    %rax,0x130(%rsp)
   cf782:	00 
   cf783:	48 8b 84 24 90 00 00 	mov    0x90(%rsp),%rax
   cf78a:	00 
   cf78b:	48 8b 8c 24 98 00 00 	mov    0x98(%rsp),%rcx
   cf792:	00 
   cf793:	0f 10 84 24 a0 00 00 	movups 0xa0(%rsp),%xmm0
   cf79a:	00 
   cf79b:	0f 10 8c 24 b0 00 00 	movups 0xb0(%rsp),%xmm1
   cf7a2:	00 
   cf7a3:	0f 29 8c 24 20 01 00 	movaps %xmm1,0x120(%rsp)
   cf7aa:	00 
   cf7ab:	0f 29 84 24 10 01 00 	movaps %xmm0,0x110(%rsp)
   cf7b2:	00 
   cf7b3:	48 89 84 24 00 01 00 	mov    %rax,0x100(%rsp)
   cf7ba:	00 
   cf7bb:	48 89 8c 24 08 01 00 	mov    %rcx,0x108(%rsp)
   cf7c2:	00 
   cf7c3:	48 8d bc 24 c8 00 00 	lea    0xc8(%rsp),%rdi
   cf7ca:	00 
   cf7cb:	48 89 ee             	mov    %rbp,%rsi
   cf7ce:	48 8b 54 24 38       	mov    0x38(%rsp),%rdx
   cf7d3:	e8 28 d9 ff ff       	call   cd100 <<hyperreal::computable::node::RadicalCoefficient>::multiply>
   cf7d8:	48 83 bc 24 c8 00 00 	cmpq   $0xffffffffffffffff,0xc8(%rsp)
   cf7df:	00 ff 
   cf7e1:	0f 84 08 05 00 00    	je     cfcef <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x66f>
   cf7e7:	48 8b 84 24 f8 00 00 	mov    0xf8(%rsp),%rax
   cf7ee:	00 
   cf7ef:	48 89 84 24 d0 01 00 	mov    %rax,0x1d0(%rsp)
   cf7f6:	00 
   cf7f7:	0f 10 84 24 c8 00 00 	movups 0xc8(%rsp),%xmm0
   cf7fe:	00 
   cf7ff:	0f 10 8c 24 d8 00 00 	movups 0xd8(%rsp),%xmm1
   cf806:	00 
   cf807:	0f 10 94 24 e8 00 00 	movups 0xe8(%rsp),%xmm2
   cf80e:	00 
   cf80f:	0f 29 94 24 c0 01 00 	movaps %xmm2,0x1c0(%rsp)
   cf816:	00 
   cf817:	0f 29 8c 24 b0 01 00 	movaps %xmm1,0x1b0(%rsp)
   cf81e:	00 
   cf81f:	0f 29 84 24 a0 01 00 	movaps %xmm0,0x1a0(%rsp)
   cf826:	00 
   cf827:	48 8d 7c 24 40       	lea    0x40(%rsp),%rdi
   cf82c:	44 89 f6             	mov    %r14d,%esi
   cf82f:	48 8d 94 24 a0 01 00 	lea    0x1a0(%rsp),%rdx
   cf836:	00 
   cf837:	e8 b4 f2 ff ff       	call   ceaf0 <<hyperreal::computable::node::PiLaurentPolynomial>::insert>
   cf83c:	84 c0                	test   %al,%al
   cf83e:	0f 84 ab 04 00 00    	je     cfcef <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x66f>
   cf844:	48 83 c3 40          	add    $0x40,%rbx
   cf848:	49 83 c5 c0          	add    $0xffffffffffffffc0,%r13
   cf84c:	0f 85 de fe ff ff    	jne    cf730 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0xb0>
   cf852:	48 8b 5c 24 60       	mov    0x60(%rsp),%rbx
   cf857:	48 89 5c 24 20       	mov    %rbx,0x20(%rsp)
   cf85c:	4c 8b 74 24 10       	mov    0x10(%rsp),%r14
   cf861:	48 8d 7c 24 18       	lea    0x18(%rsp),%rdi
   cf866:	e8 45 e1 03 00       	call   10d9b0 <<alloc::vec::into_iter::IntoIter<(i16, hyperreal::computable::node::RadicalCoefficient)> as core::ops::drop::Drop>::drop>
   cf86b:	48 8b 44 24 50       	mov    0x50(%rsp),%rax
   cf870:	49 89 46 10          	mov    %rax,0x10(%r14)
   cf874:	0f 10 44 24 40       	movups 0x40(%rsp),%xmm0
   cf879:	41 0f 11 06          	movups %xmm0,(%r14)
   cf87d:	e9 03 05 00 00       	jmp    cfd85 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x705>
   cf882:	48 8b 0e             	mov    (%rsi),%rcx
   cf885:	48 8b 46 08          	mov    0x8(%rsi),%rax
   cf889:	48 8b 56 10          	mov    0x10(%rsi),%rdx
   cf88d:	48 89 d6             	mov    %rdx,%rsi
   cf890:	48 c1 e6 06          	shl    $0x6,%rsi
   cf894:	48 01 c6             	add    %rax,%rsi
   cf897:	48 89 44 24 18       	mov    %rax,0x18(%rsp)
   cf89c:	48 89 44 24 20       	mov    %rax,0x20(%rsp)
   cf8a1:	48 89 4c 24 28       	mov    %rcx,0x28(%rsp)
   cf8a6:	48 89 b4 24 88 00 00 	mov    %rsi,0x88(%rsp)
   cf8ad:	00 
   cf8ae:	48 89 74 24 30       	mov    %rsi,0x30(%rsp)
   cf8b3:	48 85 d2             	test   %rdx,%rdx
   cf8b6:	0f 84 d1 03 00 00    	je     cfc8d <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x60d>
   cf8bc:	48 8d b4 24 98 00 00 	lea    0x98(%rsp),%rsi
   cf8c3:	00 
   cf8c4:	4c 89 e1             	mov    %r12,%rcx
   cf8c7:	48 c1 e1 06          	shl    $0x6,%rcx
   cf8cb:	48 03 0c 24          	add    (%rsp),%rcx
   cf8cf:	48 89 4c 24 60       	mov    %rcx,0x60(%rsp)
   cf8d4:	4c 89 74 24 10       	mov    %r14,0x10(%rsp)
   cf8d9:	48 8d 50 40          	lea    0x40(%rax),%rdx
   cf8dd:	48 8b 48 08          	mov    0x8(%rax),%rcx
   cf8e1:	48 83 f9 ff          	cmp    $0xffffffffffffffff,%rcx
   cf8e5:	0f 84 9d 03 00 00    	je     cfc88 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x608>
   cf8eb:	48 89 54 24 08       	mov    %rdx,0x8(%rsp)
   cf8f0:	0f b7 10             	movzwl (%rax),%edx
   cf8f3:	66 89 54 24 38       	mov    %dx,0x38(%rsp)
   cf8f8:	48 89 8c 24 90 00 00 	mov    %rcx,0x90(%rsp)
   cf8ff:	00 
   cf900:	0f 10 40 10          	movups 0x10(%rax),%xmm0
   cf904:	0f 10 48 20          	movups 0x20(%rax),%xmm1
   cf908:	0f 10 50 30          	movups 0x30(%rax),%xmm2
   cf90c:	0f 11 56 20          	movups %xmm2,0x20(%rsi)
   cf910:	0f 11 4e 10          	movups %xmm1,0x10(%rsi)
   cf914:	0f 11 06             	movups %xmm0,(%rsi)
   cf917:	4d 85 e4             	test   %r12,%r12
   cf91a:	0f 84 30 03 00 00    	je     cfc50 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x5d0>
   cf920:	4c 8b b4 24 c0 00 00 	mov    0xc0(%rsp),%r14
   cf927:	00 
   cf928:	48 8b 9c 24 a0 00 00 	mov    0xa0(%rsp),%rbx
   cf92f:	00 
   cf930:	48 89 dd             	mov    %rbx,%rbp
   cf933:	48 c1 e5 04          	shl    $0x4,%rbp
   cf937:	48 89 d8             	mov    %rbx,%rax
   cf93a:	48 c1 e8 3c          	shr    $0x3c,%rax
   cf93e:	0f 94 c0             	sete   %al
   cf941:	48 b9 f9 ff ff ff ff 	movabs $0x7ffffffffffffff9,%rcx
   cf948:	ff ff 7f 
   cf94b:	48 39 cd             	cmp    %rcx,%rbp
   cf94e:	0f 92 c1             	setb   %cl
   cf951:	84 c8                	test   %cl,%al
   cf953:	0f 84 72 03 00 00    	je     cfccb <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x64b>
   cf959:	48 85 ed             	test   %rbp,%rbp
   cf95c:	0f 84 ab 01 00 00    	je     cfb0d <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x48d>
   cf962:	48 8b 84 24 98 00 00 	mov    0x98(%rsp),%rax
   cf969:	00 
   cf96a:	48 89 84 24 40 01 00 	mov    %rax,0x140(%rsp)
   cf971:	00 
   cf972:	4c 8b 3c 24          	mov    (%rsp),%r15
   cf976:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
   cf97d:	00 00 00 
   cf980:	45 0f b7 2f          	movzwl (%r15),%r13d
   cf984:	66 44 03 6c 24 38    	add    0x38(%rsp),%r13w
   cf98a:	0f 80 7c 03 00 00    	jo     cfd0c <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x68c>
   cf990:	f0 49 ff 06          	lock incq (%r14)
   cf994:	0f 8e 53 03 00 00    	jle    cfced <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x66d>
   cf99a:	4c 89 74 24 68       	mov    %r14,0x68(%rsp)
   cf99f:	ff 15 a3 f9 09 00    	call   *0x9f9a3(%rip)        # 16f348 <_DYNAMIC+0x338>
   cf9a5:	be 08 00 00 00       	mov    $0x8,%esi
   cf9aa:	48 89 ef             	mov    %rbp,%rdi
   cf9ad:	ff 15 9d f9 09 00    	call   *0x9f99d(%rip)        # 16f350 <_DYNAMIC+0x340>
   cf9b3:	48 85 c0             	test   %rax,%rax
   cf9b6:	0f 84 55 04 00 00    	je     cfe11 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x791>
   cf9bc:	48 85 db             	test   %rbx,%rbx
   cf9bf:	74 48                	je     cfa09 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x389>
   cf9c1:	31 c9                	xor    %ecx,%ecx
   cf9c3:	48 89 da             	mov    %rbx,%rdx
   cf9c6:	48 8b b4 24 40 01 00 	mov    0x140(%rsp),%rsi
   cf9cd:	00 
   cf9ce:	66 90                	xchg   %ax,%ax
   cf9d0:	48 39 cd             	cmp    %rcx,%rbp
   cf9d3:	74 34                	je     cfa09 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x389>
   cf9d5:	48 8b 3e             	mov    (%rsi),%rdi
   cf9d8:	4c 8b 46 08          	mov    0x8(%rsi),%r8
   cf9dc:	f0 48 ff 07          	lock incq (%rdi)
   cf9e0:	0f 8e 07 03 00 00    	jle    cfced <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x66d>
   cf9e6:	f0 49 ff 00          	lock incq (%r8)
   cf9ea:	0f 8e fd 02 00 00    	jle    cfced <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x66d>
   cf9f0:	48 ff ca             	dec    %rdx
   cf9f3:	48 83 c6 10          	add    $0x10,%rsi
   cf9f7:	48 89 3c 08          	mov    %rdi,(%rax,%rcx,1)
   cf9fb:	4c 89 44 08 08       	mov    %r8,0x8(%rax,%rcx,1)
   cfa00:	48 83 c1 10          	add    $0x10,%rcx
   cfa04:	48 85 d2             	test   %rdx,%rdx
   cfa07:	75 c7                	jne    cf9d0 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x350>
   cfa09:	48 89 5c 24 70       	mov    %rbx,0x70(%rsp)
   cfa0e:	48 89 44 24 78       	mov    %rax,0x78(%rsp)
   cfa13:	48 89 9c 24 80 00 00 	mov    %rbx,0x80(%rsp)
   cfa1a:	00 
   cfa1b:	48 8d bc 24 48 01 00 	lea    0x148(%rsp),%rdi
   cfa22:	00 
   cfa23:	48 8d b4 24 a8 00 00 	lea    0xa8(%rsp),%rsi
   cfa2a:	00 
   cfa2b:	e8 70 32 04 00       	call   112ca0 <<alloc::vec::Vec<(hyperreal::computable::node::Computable, hyperreal::rational::arithmetic::Rational)> as core::clone::Clone>::clone>
   cfa30:	49 8d 57 08          	lea    0x8(%r15),%rdx
   cfa34:	4c 89 b4 24 30 01 00 	mov    %r14,0x130(%rsp)
   cfa3b:	00 
   cfa3c:	0f 10 44 24 70       	movups 0x70(%rsp),%xmm0
   cfa41:	0f 29 84 24 00 01 00 	movaps %xmm0,0x100(%rsp)
   cfa48:	00 
   cfa49:	48 8b 84 24 80 00 00 	mov    0x80(%rsp),%rax
   cfa50:	00 
   cfa51:	48 89 84 24 10 01 00 	mov    %rax,0x110(%rsp)
   cfa58:	00 
   cfa59:	48 8b 84 24 58 01 00 	mov    0x158(%rsp),%rax
   cfa60:	00 
   cfa61:	48 8d 8c 24 18 01 00 	lea    0x118(%rsp),%rcx
   cfa68:	00 
   cfa69:	48 89 41 10          	mov    %rax,0x10(%rcx)
   cfa6d:	0f 10 84 24 48 01 00 	movups 0x148(%rsp),%xmm0
   cfa74:	00 
   cfa75:	0f 11 01             	movups %xmm0,(%rcx)
   cfa78:	48 8d bc 24 c8 00 00 	lea    0xc8(%rsp),%rdi
   cfa7f:	00 
   cfa80:	48 8d b4 24 00 01 00 	lea    0x100(%rsp),%rsi
   cfa87:	00 
   cfa88:	e8 73 d6 ff ff       	call   cd100 <<hyperreal::computable::node::RadicalCoefficient>::multiply>
   cfa8d:	48 83 bc 24 c8 00 00 	cmpq   $0xffffffffffffffff,0xc8(%rsp)
   cfa94:	00 ff 
   cfa96:	0f 84 70 02 00 00    	je     cfd0c <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x68c>
   cfa9c:	48 8b 84 24 f8 00 00 	mov    0xf8(%rsp),%rax
   cfaa3:	00 
   cfaa4:	48 89 84 24 90 01 00 	mov    %rax,0x190(%rsp)
   cfaab:	00 
   cfaac:	0f 10 84 24 c8 00 00 	movups 0xc8(%rsp),%xmm0
   cfab3:	00 
   cfab4:	0f 10 8c 24 d8 00 00 	movups 0xd8(%rsp),%xmm1
   cfabb:	00 
   cfabc:	0f 10 94 24 e8 00 00 	movups 0xe8(%rsp),%xmm2
   cfac3:	00 
   cfac4:	0f 29 94 24 80 01 00 	movaps %xmm2,0x180(%rsp)
   cfacb:	00 
   cfacc:	0f 29 8c 24 70 01 00 	movaps %xmm1,0x170(%rsp)
   cfad3:	00 
   cfad4:	0f 29 84 24 60 01 00 	movaps %xmm0,0x160(%rsp)
   cfadb:	00 
   cfadc:	48 8d 7c 24 40       	lea    0x40(%rsp),%rdi
   cfae1:	44 89 ee             	mov    %r13d,%esi
   cfae4:	48 8d 94 24 60 01 00 	lea    0x160(%rsp),%rdx
   cfaeb:	00 
   cfaec:	e8 ff ef ff ff       	call   ceaf0 <<hyperreal::computable::node::PiLaurentPolynomial>::insert>
   cfaf1:	84 c0                	test   %al,%al
   cfaf3:	0f 84 13 02 00 00    	je     cfd0c <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x68c>
   cfaf9:	49 83 c7 40          	add    $0x40,%r15
   cfafd:	4c 3b 7c 24 60       	cmp    0x60(%rsp),%r15
   cfb02:	0f 85 78 fe ff ff    	jne    cf980 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x300>
   cfb08:	e9 43 01 00 00       	jmp    cfc50 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x5d0>
   cfb0d:	4c 8b 2c 24          	mov    (%rsp),%r13
   cfb11:	66 66 66 66 66 66 2e 	data16 data16 data16 data16 data16 cs nopw 0x0(%rax,%rax,1)
   cfb18:	0f 1f 84 00 00 00 00 
   cfb1f:	00 
   cfb20:	41 0f b7 5d 00       	movzwl 0x0(%r13),%ebx
   cfb25:	66 03 5c 24 38       	add    0x38(%rsp),%bx
   cfb2a:	0f 80 dc 01 00 00    	jo     cfd0c <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x68c>
   cfb30:	f0 49 ff 06          	lock incq (%r14)
   cfb34:	0f 8e b3 01 00 00    	jle    cfced <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x66d>
   cfb3a:	4c 89 74 24 68       	mov    %r14,0x68(%rsp)
   cfb3f:	48 c7 44 24 70 00 00 	movq   $0x0,0x70(%rsp)
   cfb46:	00 00 
   cfb48:	48 c7 44 24 78 08 00 	movq   $0x8,0x78(%rsp)
   cfb4f:	00 00 
   cfb51:	48 c7 84 24 80 00 00 	movq   $0x0,0x80(%rsp)
   cfb58:	00 00 00 00 00 
   cfb5d:	48 8d bc 24 48 01 00 	lea    0x148(%rsp),%rdi
   cfb64:	00 
   cfb65:	48 8d b4 24 a8 00 00 	lea    0xa8(%rsp),%rsi
   cfb6c:	00 
   cfb6d:	e8 2e 31 04 00       	call   112ca0 <<alloc::vec::Vec<(hyperreal::computable::node::Computable, hyperreal::rational::arithmetic::Rational)> as core::clone::Clone>::clone>
   cfb72:	4c 89 b4 24 30 01 00 	mov    %r14,0x130(%rsp)
   cfb79:	00 
   cfb7a:	0f 10 44 24 70       	movups 0x70(%rsp),%xmm0
   cfb7f:	0f 29 84 24 00 01 00 	movaps %xmm0,0x100(%rsp)
   cfb86:	00 
   cfb87:	48 8b 84 24 80 00 00 	mov    0x80(%rsp),%rax
   cfb8e:	00 
   cfb8f:	48 89 84 24 10 01 00 	mov    %rax,0x110(%rsp)
   cfb96:	00 
   cfb97:	48 8b 84 24 58 01 00 	mov    0x158(%rsp),%rax
   cfb9e:	00 
   cfb9f:	48 8d 8c 24 18 01 00 	lea    0x118(%rsp),%rcx
   cfba6:	00 
   cfba7:	48 89 41 10          	mov    %rax,0x10(%rcx)
   cfbab:	0f 10 84 24 48 01 00 	movups 0x148(%rsp),%xmm0
   cfbb2:	00 
   cfbb3:	0f 11 01             	movups %xmm0,(%rcx)
   cfbb6:	49 83 c5 08          	add    $0x8,%r13
   cfbba:	48 8d bc 24 c8 00 00 	lea    0xc8(%rsp),%rdi
   cfbc1:	00 
   cfbc2:	48 8d b4 24 00 01 00 	lea    0x100(%rsp),%rsi
   cfbc9:	00 
   cfbca:	4c 89 ea             	mov    %r13,%rdx
   cfbcd:	e8 2e d5 ff ff       	call   cd100 <<hyperreal::computable::node::RadicalCoefficient>::multiply>
   cfbd2:	48 83 bc 24 c8 00 00 	cmpq   $0xffffffffffffffff,0xc8(%rsp)
   cfbd9:	00 ff 
   cfbdb:	0f 84 2b 01 00 00    	je     cfd0c <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x68c>
   cfbe1:	48 8b 84 24 f8 00 00 	mov    0xf8(%rsp),%rax
   cfbe8:	00 
   cfbe9:	48 89 84 24 90 01 00 	mov    %rax,0x190(%rsp)
   cfbf0:	00 
   cfbf1:	0f 10 84 24 c8 00 00 	movups 0xc8(%rsp),%xmm0
   cfbf8:	00 
   cfbf9:	0f 10 8c 24 d8 00 00 	movups 0xd8(%rsp),%xmm1
   cfc00:	00 
   cfc01:	0f 10 94 24 e8 00 00 	movups 0xe8(%rsp),%xmm2
   cfc08:	00 
   cfc09:	0f 29 94 24 80 01 00 	movaps %xmm2,0x180(%rsp)
   cfc10:	00 
   cfc11:	0f 29 8c 24 70 01 00 	movaps %xmm1,0x170(%rsp)
   cfc18:	00 
   cfc19:	0f 29 84 24 60 01 00 	movaps %xmm0,0x160(%rsp)
   cfc20:	00 
   cfc21:	48 8d 7c 24 40       	lea    0x40(%rsp),%rdi
   cfc26:	89 de                	mov    %ebx,%esi
   cfc28:	48 8d 94 24 60 01 00 	lea    0x160(%rsp),%rdx
   cfc2f:	00 
   cfc30:	e8 bb ee ff ff       	call   ceaf0 <<hyperreal::computable::node::PiLaurentPolynomial>::insert>
   cfc35:	84 c0                	test   %al,%al
   cfc37:	0f 84 cf 00 00 00    	je     cfd0c <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x68c>
   cfc3d:	49 83 c5 38          	add    $0x38,%r13
   cfc41:	4c 3b 6c 24 60       	cmp    0x60(%rsp),%r13
   cfc46:	0f 85 d4 fe ff ff    	jne    cfb20 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x4a0>
   cfc4c:	0f 1f 40 00          	nopl   0x0(%rax)
   cfc50:	48 8d bc 24 90 00 00 	lea    0x90(%rsp),%rdi
   cfc57:	00 
   cfc58:	e8 f3 10 fe ff       	call   b0d50 <core::ptr::drop_glue::<hyperreal::computable::node::RadicalCoefficient>>
   cfc5d:	48 8b 4c 24 08       	mov    0x8(%rsp),%rcx
   cfc62:	48 89 c8             	mov    %rcx,%rax
   cfc65:	48 3b 8c 24 88 00 00 	cmp    0x88(%rsp),%rcx
   cfc6c:	00 
   cfc6d:	4c 8b 74 24 10       	mov    0x10(%rsp),%r14
   cfc72:	48 8d b4 24 98 00 00 	lea    0x98(%rsp),%rsi
   cfc79:	00 
   cfc7a:	0f 85 59 fc ff ff    	jne    cf8d9 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x259>
   cfc80:	48 8b 94 24 88 00 00 	mov    0x88(%rsp),%rdx
   cfc87:	00 
   cfc88:	48 89 54 24 20       	mov    %rdx,0x20(%rsp)
   cfc8d:	48 8d 7c 24 18       	lea    0x18(%rsp),%rdi
   cfc92:	e8 19 dd 03 00       	call   10d9b0 <<alloc::vec::into_iter::IntoIter<(i16, hyperreal::computable::node::RadicalCoefficient)> as core::ops::drop::Drop>::drop>
   cfc97:	48 8b 44 24 50       	mov    0x50(%rsp),%rax
   cfc9c:	49 89 46 10          	mov    %rax,0x10(%r14)
   cfca0:	0f 10 44 24 40       	movups 0x40(%rsp),%xmm0
   cfca5:	41 0f 11 06          	movups %xmm0,(%r14)
   cfca9:	49 ff c4             	inc    %r12
   cfcac:	48 8b 3c 24          	mov    (%rsp),%rdi
   cfcb0:	49 83 fc 01          	cmp    $0x1,%r12
   cfcb4:	0f 84 ed 00 00 00    	je     cfda7 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x727>
   cfcba:	48 8d 5f 40          	lea    0x40(%rdi),%rbx
   cfcbe:	49 ff cc             	dec    %r12
   cfcc1:	e8 5a 1d 04 00       	call   111a20 <core::ptr::drop_glue::<(i16, hyperreal::computable::node::RadicalCoefficient)>>
   cfcc6:	48 89 df             	mov    %rbx,%rdi
   cfcc9:	eb e5                	jmp    cfcb0 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x630>
   cfccb:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
   cfcd0:	48 89 44 24 20       	mov    %rax,0x20(%rsp)
   cfcd5:	48 8b 04 24          	mov    (%rsp),%rax
   cfcd9:	0f b7 4c 24 38       	movzwl 0x38(%rsp),%ecx
   cfcde:	66 03 08             	add    (%rax),%cx
   cfce1:	70 33                	jo     cfd16 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x696>
   cfce3:	f0 49 ff 06          	lock incq (%r14)
   cfce7:	0f 8f ec 00 00 00    	jg     cfdd9 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x759>
   cfced:	0f 0b                	ud2
   cfcef:	48 89 5c 24 20       	mov    %rbx,0x20(%rsp)
   cfcf4:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
   cfcf9:	48 c7 00 ff ff ff ff 	movq   $0xffffffffffffffff,(%rax)
   cfd00:	48 8d 7c 24 18       	lea    0x18(%rsp),%rdi
   cfd05:	e8 a6 dc 03 00       	call   10d9b0 <<alloc::vec::into_iter::IntoIter<(i16, hyperreal::computable::node::RadicalCoefficient)> as core::ops::drop::Drop>::drop>
   cfd0a:	eb 2d                	jmp    cfd39 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x6b9>
   cfd0c:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
   cfd11:	48 89 44 24 20       	mov    %rax,0x20(%rsp)
   cfd16:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
   cfd1b:	48 c7 00 ff ff ff ff 	movq   $0xffffffffffffffff,(%rax)
   cfd22:	48 8d bc 24 90 00 00 	lea    0x90(%rsp),%rdi
   cfd29:	00 
   cfd2a:	e8 21 10 fe ff       	call   b0d50 <core::ptr::drop_glue::<hyperreal::computable::node::RadicalCoefficient>>
   cfd2f:	48 8d 7c 24 18       	lea    0x18(%rsp),%rdi
   cfd34:	e8 77 dc 03 00       	call   10d9b0 <<alloc::vec::into_iter::IntoIter<(i16, hyperreal::computable::node::RadicalCoefficient)> as core::ops::drop::Drop>::drop>
   cfd39:	48 8b 7c 24 48       	mov    0x48(%rsp),%rdi
   cfd3e:	4c 8b 74 24 50       	mov    0x50(%rsp),%r14
   cfd43:	49 ff c6             	inc    %r14
   cfd46:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
   cfd4d:	00 00 00 
   cfd50:	49 83 fe 01          	cmp    $0x1,%r14
   cfd54:	74 11                	je     cfd67 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x6e7>
   cfd56:	48 8d 5f 40          	lea    0x40(%rdi),%rbx
   cfd5a:	49 ff ce             	dec    %r14
   cfd5d:	e8 be 1c 04 00       	call   111a20 <core::ptr::drop_glue::<(i16, hyperreal::computable::node::RadicalCoefficient)>>
   cfd62:	48 89 df             	mov    %rbx,%rdi
   cfd65:	eb e9                	jmp    cfd50 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x6d0>
   cfd67:	48 8b 74 24 40       	mov    0x40(%rsp),%rsi
   cfd6c:	48 85 f6             	test   %rsi,%rsi
   cfd6f:	74 14                	je     cfd85 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x705>
   cfd71:	48 8b 7c 24 48       	mov    0x48(%rsp),%rdi
   cfd76:	48 c1 e6 06          	shl    $0x6,%rsi
   cfd7a:	ba 08 00 00 00       	mov    $0x8,%edx
   cfd7f:	ff 15 4b f5 09 00    	call   *0x9f54b(%rip)        # 16f2d0 <_DYNAMIC+0x2c0>
   cfd85:	49 ff c4             	inc    %r12
   cfd88:	48 8b 3c 24          	mov    (%rsp),%rdi
   cfd8c:	0f 1f 40 00          	nopl   0x0(%rax)
   cfd90:	49 83 fc 01          	cmp    $0x1,%r12
   cfd94:	74 11                	je     cfda7 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x727>
   cfd96:	48 8d 5f 40          	lea    0x40(%rdi),%rbx
   cfd9a:	49 ff cc             	dec    %r12
   cfd9d:	e8 7e 1c 04 00       	call   111a20 <core::ptr::drop_glue::<(i16, hyperreal::computable::node::RadicalCoefficient)>>
   cfda2:	48 89 df             	mov    %rbx,%rdi
   cfda5:	eb e9                	jmp    cfd90 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x710>
   cfda7:	48 8b 44 24 58       	mov    0x58(%rsp),%rax
   cfdac:	48 8b 30             	mov    (%rax),%rsi
   cfdaf:	48 85 f6             	test   %rsi,%rsi
   cfdb2:	74 13                	je     cfdc7 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x747>
   cfdb4:	48 c1 e6 06          	shl    $0x6,%rsi
   cfdb8:	ba 08 00 00 00       	mov    $0x8,%edx
   cfdbd:	48 8b 3c 24          	mov    (%rsp),%rdi
   cfdc1:	ff 15 09 f5 09 00    	call   *0x9f509(%rip)        # 16f2d0 <_DYNAMIC+0x2c0>
   cfdc7:	48 81 c4 d8 01 00 00 	add    $0x1d8,%rsp
   cfdce:	5b                   	pop    %rbx
   cfdcf:	41 5c                	pop    %r12
   cfdd1:	41 5d                	pop    %r13
   cfdd3:	41 5e                	pop    %r14
   cfdd5:	41 5f                	pop    %r15
   cfdd7:	5d                   	pop    %rbp
   cfdd8:	c3                   	ret
   cfdd9:	4c 89 74 24 68       	mov    %r14,0x68(%rsp)
   cfdde:	31 ff                	xor    %edi,%edi
   cfde0:	48 89 ee             	mov    %rbp,%rsi
   cfde3:	ff 15 57 f5 09 00    	call   *0x9f557(%rip)        # 16f340 <_DYNAMIC+0x330>
   cfde9:	e9 ff fe ff ff       	jmp    cfced <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x66d>
   cfdee:	48 89 5c 24 20       	mov    %rbx,0x20(%rsp)
   cfdf3:	48 8b 44 24 10       	mov    0x10(%rsp),%rax
   cfdf8:	48 c7 00 ff ff ff ff 	movq   $0xffffffffffffffff,(%rax)
   cfdff:	48 8d bc 24 90 00 00 	lea    0x90(%rsp),%rdi
   cfe06:	00 
   cfe07:	e8 44 0f fe ff       	call   b0d50 <core::ptr::drop_glue::<hyperreal::computable::node::RadicalCoefficient>>
   cfe0c:	e9 ef fe ff ff       	jmp    cfd00 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x680>
   cfe11:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
   cfe16:	48 89 44 24 20       	mov    %rax,0x20(%rsp)
   cfe1b:	bf 08 00 00 00       	mov    $0x8,%edi
   cfe20:	eb be                	jmp    cfde0 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x760>
   cfe22:	49 89 c7             	mov    %rax,%r15
   cfe25:	eb 5c                	jmp    cfe83 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x803>
   cfe27:	49 89 c7             	mov    %rax,%r15
   cfe2a:	e9 56 01 00 00       	jmp    cff85 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x905>
   cfe2f:	49 89 c7             	mov    %rax,%r15
   cfe32:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
   cfe37:	48 89 44 24 20       	mov    %rax,0x20(%rsp)
   cfe3c:	e9 44 01 00 00       	jmp    cff85 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x905>
   cfe41:	eb 08                	jmp    cfe4b <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x7cb>
   cfe43:	49 89 c7             	mov    %rax,%r15
   cfe46:	e9 44 01 00 00       	jmp    cff8f <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x90f>
   cfe4b:	49 89 c7             	mov    %rax,%r15
   cfe4e:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
   cfe53:	48 89 44 24 20       	mov    %rax,0x20(%rsp)
   cfe58:	48 8d 7c 24 70       	lea    0x70(%rsp),%rdi
   cfe5d:	e8 be fb fd ff       	call   afa20 <core::ptr::drop_glue::<alloc::vec::Vec<(hyperreal::rational::arithmetic::Rational, hyperreal::rational::arithmetic::Rational)>>>
   cfe62:	e9 00 01 00 00       	jmp    cff67 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x8e7>
   cfe67:	eb 00                	jmp    cfe69 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x7e9>
   cfe69:	49 89 c7             	mov    %rax,%r15
   cfe6c:	48 8b 44 24 08       	mov    0x8(%rsp),%rax
   cfe71:	48 89 44 24 20       	mov    %rax,0x20(%rsp)
   cfe76:	e9 fd 00 00 00       	jmp    cff78 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x8f8>
   cfe7b:	49 89 c7             	mov    %rax,%r15
   cfe7e:	48 89 5c 24 20       	mov    %rbx,0x20(%rsp)
   cfe83:	48 8d 7c 24 18       	lea    0x18(%rsp),%rdi
   cfe88:	e8 23 db 03 00       	call   10d9b0 <<alloc::vec::into_iter::IntoIter<(i16, hyperreal::computable::node::RadicalCoefficient)> as core::ops::drop::Drop>::drop>
   cfe8d:	e9 fd 00 00 00       	jmp    cff8f <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x90f>
   cfe92:	49 89 c7             	mov    %rax,%r15
   cfe95:	66 66 2e 0f 1f 84 00 	data16 cs nopw 0x0(%rax,%rax,1)
   cfe9c:	00 00 00 00 
   cfea0:	49 ff cc             	dec    %r12
   cfea3:	0f 84 8d 00 00 00    	je     cff36 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x8b6>
   cfea9:	4c 8d 73 40          	lea    0x40(%rbx),%r14
   cfead:	48 89 df             	mov    %rbx,%rdi
   cfeb0:	e8 6b 1b 04 00       	call   111a20 <core::ptr::drop_glue::<(i16, hyperreal::computable::node::RadicalCoefficient)>>
   cfeb5:	4c 89 f3             	mov    %r14,%rbx
   cfeb8:	eb e6                	jmp    cfea0 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x820>
   cfeba:	ff 15 f8 f3 09 00    	call   *0x9f3f8(%rip)        # 16f2b8 <_DYNAMIC+0x2a8>
   cfec0:	49 89 c7             	mov    %rax,%r15
   cfec3:	66 66 66 66 2e 0f 1f 	data16 data16 data16 cs nopw 0x0(%rax,%rax,1)
   cfeca:	84 00 00 00 00 00 
   cfed0:	49 ff ce             	dec    %r14
   cfed3:	74 11                	je     cfee6 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x866>
   cfed5:	4c 8d 63 40          	lea    0x40(%rbx),%r12
   cfed9:	48 89 df             	mov    %rbx,%rdi
   cfedc:	e8 3f 1b 04 00       	call   111a20 <core::ptr::drop_glue::<(i16, hyperreal::computable::node::RadicalCoefficient)>>
   cfee1:	4c 89 e3             	mov    %r12,%rbx
   cfee4:	eb ea                	jmp    cfed0 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x850>
   cfee6:	48 8b 74 24 40       	mov    0x40(%rsp),%rsi
   cfeeb:	48 85 f6             	test   %rsi,%rsi
   cfeee:	0f 84 a5 00 00 00    	je     cff99 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x919>
   cfef4:	48 8b 7c 24 48       	mov    0x48(%rsp),%rdi
   cfef9:	48 c1 e6 06          	shl    $0x6,%rsi
   cfefd:	ba 08 00 00 00       	mov    $0x8,%edx
   cff02:	ff 15 c8 f3 09 00    	call   *0x9f3c8(%rip)        # 16f2d0 <_DYNAMIC+0x2c0>
   cff08:	e9 8c 00 00 00       	jmp    cff99 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x919>
   cff0d:	ff 15 a5 f3 09 00    	call   *0x9f3a5(%rip)        # 16f2b8 <_DYNAMIC+0x2a8>
   cff13:	49 89 c7             	mov    %rax,%r15
   cff16:	66 2e 0f 1f 84 00 00 	cs nopw 0x0(%rax,%rax,1)
   cff1d:	00 00 00 
   cff20:	49 ff cc             	dec    %r12
   cff23:	74 11                	je     cff36 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x8b6>
   cff25:	4c 8d 73 40          	lea    0x40(%rbx),%r14
   cff29:	48 89 df             	mov    %rbx,%rdi
   cff2c:	e8 ef 1a 04 00       	call   111a20 <core::ptr::drop_glue::<(i16, hyperreal::computable::node::RadicalCoefficient)>>
   cff31:	4c 89 f3             	mov    %r14,%rbx
   cff34:	eb ea                	jmp    cff20 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x8a0>
   cff36:	48 8b 44 24 58       	mov    0x58(%rsp),%rax
   cff3b:	48 8b 30             	mov    (%rax),%rsi
   cff3e:	48 85 f6             	test   %rsi,%rsi
   cff41:	74 60                	je     cffa3 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x923>
   cff43:	48 c1 e6 06          	shl    $0x6,%rsi
   cff47:	ba 08 00 00 00       	mov    $0x8,%edx
   cff4c:	48 8b 3c 24          	mov    (%rsp),%rdi
   cff50:	ff 15 7a f3 09 00    	call   *0x9f37a(%rip)        # 16f2d0 <_DYNAMIC+0x2c0>
   cff56:	4c 89 ff             	mov    %r15,%rdi
   cff59:	e8 22 90 09 00       	call   168f80 <_Unwind_Resume@plt>
   cff5e:	ff 15 54 f3 09 00    	call   *0x9f354(%rip)        # 16f2b8 <_DYNAMIC+0x2a8>
   cff64:	49 89 c7             	mov    %rax,%r15
   cff67:	f0 49 ff 0e          	lock decq (%r14)
   cff6b:	75 0b                	jne    cff78 <<hyperreal::computable::node::PiLaurentPolynomial>::multiply+0x8f8>
   cff6d:	48 8d 7c 24 68       	lea    0x68(%rsp),%rdi
   cff72:	ff 15 38 f3 09 00    	call   *0x9f338(%rip)        # 16f2b0 <_DYNAMIC+0x2a0>
   cff78:	48 8d bc 24 90 00 00 	lea    0x90(%rsp),%rdi
   cff7f:	00 
   cff80:	e8 cb 0d fe ff       	call   b0d50 <core::ptr::drop_glue::<hyperreal::computable::node::RadicalCoefficient>>
   cff85:	48 8d 7c 24 18       	lea    0x18(%rsp),%rdi
   cff8a:	e8 21 da 03 00       	call   10d9b0 <<alloc::vec::into_iter::IntoIter<(i16, hyperreal::computable::node::RadicalCoefficient)> as core::ops::drop::Drop>::drop>
   cff8f:	48 8d 7c 24 40       	lea    0x40(%rsp),%rdi
   cff94:	e8 57 fb fd ff       	call   afaf0 <core::ptr::drop_glue::<alloc::vec::Vec<(i16, hyperreal::computable::node::RadicalCoefficient)>>>
   cff99:	48 8b 7c 24 58       	mov    0x58(%rsp),%rdi
   cff9e:	e8 4d fb fd ff       	call   afaf0 <core::ptr::drop_glue::<alloc::vec::Vec<(i16, hyperreal::computable::node::RadicalCoefficient)>>>
   cffa3:	4c 89 ff             	mov    %r15,%rdi
   cffa6:	e8 d5 8f 09 00       	call   168f80 <_Unwind_Resume@plt>
   cffab:	ff 15 07 f3 09 00    	call   *0x9f307(%rip)        # 16f2b8 <_DYNAMIC+0x2a8>
   cffb1:	ff 15 01 f3 09 00    	call   *0x9f301(%rip)        # 16f2b8 <_DYNAMIC+0x2a8>
