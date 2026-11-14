# encoding: ascii-8bit

Gem::Specification.new do |s|
  s.name = 'openc3-cosmos-orthostream'
  s.summary = 'OrthoStream OpenC3 COSMOS plugin'
  s.description = 'Commands, telemetry and screens of the OrthoStream cFS mission, generated from its EDS definitions.'
  s.license = 'Apache-2.0'
  s.authors = ['OrthoStream']
  s.homepage = 'https://github.com/'
  s.platform = Gem::Platform::RUBY
  s.version = ENV['VERSION'] ? ENV['VERSION'].dup : '0.1.0'
  s.files = Dir.glob("{targets,lib,tools,microservices}/**/*") + %w(Rakefile README.md LICENSE.txt plugin.txt)
end
