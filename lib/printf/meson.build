project('printf', 'c', version: '4.0.0')

conf = configuration_data()
conf.set('PRINTF_NTOA_BUFFER_SIZE', get_option('ntoa-buffer-size'))
conf.set('PRINTF_FTOA_BUFFER_SIZE', get_option('ftoa-buffer-size'))
conf.set('PRINTF_DISABLE_SUPPORT_FLOAT', not get_option('support-float'))
conf.set('PRINTF_DISABLE_SUPPORT_EXPONENTIAL', not get_option('support-exponential'))
conf.set('PRINTF_DEFAULT_FLOAT_PRECISION', get_option('float-precision'))
conf.set('PRINTF_MAX_FLOAT', get_option('max-float'))
conf.set('PRINTF_DISABLE_SUPPORT_LONG_LONG', not get_option('support-long-long'))
conf.set('PRINTF_DISABLE_SUPPORT_PTRDIFF_T', not get_option('support-ptrdiff-t'))
configure_file(output: 'printf_config.h', configuration: conf)

printf_dep = declare_dependency(
  sources: files('printf.c'),
  include_directories: include_directories('.'),
  compile_args: ['-DPRINTF_INCLUDE_CONFIG_H'],
)
