#!/bin/sh

DEFFILE=$(basename $1)
if [ "$DEFFILE" = "defconfig" ]; then

	PARENT=$(grep -P '^#include\s+\"\S+\"' $1 | awk '{print $2}' | sed 's/\"//g')
	RELATIVE=$(echo $1 | sed "s|defconfig|$PARENT|")

	if [ -f $RELATIVE ]; then
		ABSREL=$(readlink -m $RELATIVE)
		perl - $ABSREL $1 <<'__HERE__'
		my $file = shift;
		my $deff = shift;
		my $out = "${deff}.tmp";
		open(OUT,">$out") or die "$!: $out\n";
		open my $fh, '<', $file or die "error opening $file: $!";
		my $pdata = do { local $/; <$fh> };
		open my $fh1, '<', $deff or die "error opening $deff: $!";
		while(<$fh1>) {
			$_ =~ s/#include \"\S*defconfig\"//;
			if (/^# (CONFIG\S+) is not set/) {
			$pdata =~ s/$1=y//;
			}
			$data .= $_;
		}
		print OUT $pdata . "\n";
        print OUT $data . "\n";
		close(OUT);
__HERE__
		# Helps to keep source clean in overlay build by breaking the link
		# Does not affect to conventional builds
		mv ${1}.tmp $1
	fi

fi
