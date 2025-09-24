FROM --platform=$BUILDPLATFORM alpine:3.19 AS builder

# Build dependencies
RUN apk add --no-cache \
    build-base \
    linux-headers \
    glib-dev \
    pixman-dev \
    zlib-dev \
    bzip2-dev \
    libpng-dev \
    libjpeg-turbo-dev \
    gnutls-dev \
    nettle-dev \
    curl \
    git \
    bash \
    ninja

# Clone and build QEMU
ARG QEMU_VERSION=9.2.2
RUN git clone --branch v${QEMU_VERSION} https://git.qemu.org/git/qemu.git /qemu && \
    cd /qemu && \
    ./configure --target-list=aarch64-softmmu --disable-user --disable-tools --disable-docs && \
    make -j$(nproc) && \
    make install

# Final stage
FROM --platform=$TARGETPLATFORM alpine:3.19

# Runtime dependencies
RUN apk add --no-cache \
    glib \
    pixman \
    zlib \
    bzip2 \
    libpng \
    libjpeg \
    gnutls \
    nettle \
    bash \
    libbz2

COPY --from=builder /usr/local/bin/qemu-system-aarch64 /usr/local/bin/

ENV QEMU_AUDIO_DRV=none

RUN adduser -D -u 1000 qemu
USER qemu

ENTRYPOINT ["bash"]
